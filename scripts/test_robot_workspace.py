"""배포물에서 누락·변조·개발 PC 바이너리를 구분한다."""
import importlib.util
import json
from pathlib import Path
import subprocess
import tarfile

import pytest

spec = importlib.util.spec_from_file_location('robot_workspace', Path(__file__).with_name('robot_workspace.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def git(repo, *args):
    return subprocess.run(['git', *args], cwd=repo, check=True, capture_output=True, text=True)


def repository(tmp_path):
    repo = tmp_path / 'repo'
    repo.mkdir()
    git(repo, 'init', '-q')
    for name, value in {
        'src/mapping_robot/package.xml': '<package>robot</package>',
        'src/mapping_semantic/package.xml': '<package>semantic</package>',
        'src/mapping_gazebo/world.sdf': 'simulator',
        'install/native-x86.so': 'wrong architecture',
        'docs/robot-deployment.md': 'procedure',
    }.items():
        path = repo / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(value)
    for name in module.BUNDLE_REQUIRED_FILES:
        path = repo / name
        path.parent.mkdir(parents=True, exist_ok=True)
        if not path.exists():
            path.write_text('runtime fixture')
    git(repo, 'add', '.')
    git(repo, '-c', 'user.name=검증', '-c', 'user.email=test@example.invalid', 'commit', '-qm', '배포 검증 fixture')
    return repo


def test_bundle_contains_target_source_and_detects_tampering(tmp_path):
    repo = repository(tmp_path)
    artifact = tmp_path / 'bundle.tar.gz'
    result = module.bundle_workspace(repo, artifact)
    assert result['runtime_packages'] == ['mapping_robot', 'mapping_semantic']
    with tarfile.open(artifact) as archive:
        names = archive.getnames()
        assert not any('install/' in name or 'mapping_gazebo/' in name for name in names)
        assert all(not name.startswith('/') and '..' not in Path(name).parts for name in names)
        archive.extractall(tmp_path / 'extracted')  # 이 테스트에서 생성한 알려진 archive만 사용
    root = next((tmp_path / 'extracted').iterdir())
    assert module.verify_bundle(root)['bundle_valid']
    (root / 'src/mapping_robot/package.xml').write_text('tampered')
    assert not module.verify_bundle(root)['bundle_valid']
    with pytest.raises(ValueError, match='새 경로'):
        module.bundle_workspace(repo, artifact)


def test_bundle_rejects_uncommitted_or_missing_runtime_package(tmp_path):
    repo = repository(tmp_path)
    (repo / 'unreviewed.txt').write_text('change')
    with pytest.raises(ValueError, match='clean'):
        module.bundle_workspace(repo, tmp_path / 'dirty.tar.gz')
    (repo / 'unreviewed.txt').unlink()
    git(repo, 'rm', 'src/mapping_robot/package.xml')
    git(repo, '-c', 'user.name=검증', '-c', 'user.email=test@example.invalid', 'commit', '-qm', '필수 패키지 제거')
    with pytest.raises(ValueError, match='패키지'):
        module.bundle_workspace(repo, tmp_path / 'missing.tar.gz')


def test_bundle_rejects_manifest_path_escape(tmp_path):
    root = tmp_path / 'bundle'
    root.mkdir()
    (root / 'bundle_manifest.json').write_text(json.dumps({
        'schema_version': 1, 'git_commit': 'fixture', 'runtime_packages': list(module.RUNTIME_PACKAGES),
        'file_sha256': {**{name: '123' for name in module.BUNDLE_REQUIRED_FILES}, '../outside': '123'},
    }))
    assert not module.verify_bundle(root)['bundle_valid']


def test_environment_reports_os_python_ros_and_missing_packages_separately():
    issues = module.environment_issues({'ID': 'ubuntu', 'VERSION_ID': '20.04'},
                                       'aarch64', (3, 8), 'foxy', ['cv_bridge'])
    assert len(issues) == 4
    assert module.environment_issues({'ID': 'ubuntu', 'VERSION_ID': '22.04'},
                                      'aarch64', (3, 10), 'humble', []) == []


def test_resource_recording_is_bounded_and_never_overwrites(tmp_path):
    target = tmp_path / 'resources'
    result = module.sample_resources(target, .01, .01)
    assert result['status'] == 'completed'
    assert result['sample_count'] >= 1
    rows = [json.loads(line) for line in (target / 'resources.jsonl').read_text().splitlines()]
    assert rows[-1]['monotonic_ns'] >= rows[0]['monotonic_ns']
    assert json.loads((target / 'summary.json').read_text())['limitations']
    with pytest.raises(FileExistsError):
        module.sample_resources(target, .01, .01)


@pytest.mark.parametrize('value', ['nan', 'inf', '-1', '0'])
def test_invalid_duration(value):
    with pytest.raises(Exception):
        module.positive(value)


def test_bundle_rejects_empty_inventory_and_extra_executable(tmp_path):
    root = tmp_path / 'empty'
    root.mkdir()
    (root / 'bundle_manifest.json').write_text(json.dumps({
        'schema_version': 1, 'git_commit': 'fixture', 'runtime_packages': list(module.RUNTIME_PACKAGES),
        'file_sha256': {},
    }))
    assert not module.verify_bundle(root)['bundle_valid']
    repo = repository(tmp_path)
    artifact = tmp_path / 'bundle.tar.gz'
    module.bundle_workspace(repo, artifact)
    with tarfile.open(artifact) as archive:
        archive.extractall(tmp_path / 'extracted')
    unpacked = next((tmp_path / 'extracted').iterdir())
    (unpacked / 'src/mapping_robot/mapping_robot/injected.py').write_text('unexpected')
    assert not module.verify_bundle(unpacked)['bundle_valid']


def test_excluded_development_links_point_to_source_commit():
    text = b'[sim](../src/mapping_gazebo/README.md#world) [robot](../src/mapping_robot/README.md)'
    included = {'src/mapping_robot/README.md'}
    result = module.bundle_document('docs/robot.md', text, included, 'abcdef').decode()
    assert 'blob/abcdef/src/mapping_gazebo/README.md#world' in result
    assert '[robot](../src/mapping_robot/README.md)' in result
    assert module.bundle_document('src/module.py', b'unchanged', included, 'abcdef') == b'unchanged'


def test_bundle_reads_committed_bytes_even_when_worktree_change_is_hidden(tmp_path):
    repo = repository(tmp_path)
    path = 'src/mapping_robot/mapping_robot/profile.py'
    git(repo, 'update-index', '--assume-unchanged', path)
    (repo / path).write_text('hidden unreviewed code')
    archive_path = tmp_path / 'source.tar.gz'
    module.bundle_workspace(repo, archive_path)
    with tarfile.open(archive_path) as archive:
        name = next(n for n in archive.getnames() if n.endswith('/' + path))
        assert archive.extractfile(name).read() == b'runtime fixture'
