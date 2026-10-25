#!/usr/bin/env python3
"""실물용 소스 묶음·호스트 환경·자원 기록. 드라이버와 구동기는 실행하지 않는다."""

import argparse
from datetime import datetime, timezone
import hashlib
import io
import json
import os
from pathlib import Path
import platform
import posixpath
import re
import shutil
import subprocess
import sys
import tarfile
import time
from urllib.parse import quote


RUNTIME_PACKAGES = ('mapping_robot', 'mapping_semantic')
ROS_PACKAGES = ('mapping_robot', 'mapping_semantic', 'slam_toolbox',
                'nav2_map_server', 'cv_bridge', 'vision_msgs', 'tf2_ros',
                'ros2bag', 'rosbag2_py', 'rosbag2_storage_default_plugins')
BUNDLE_REQUIRED_FILES = (
    'src/mapping_robot/package.xml', 'src/mapping_robot/setup.py',
    'src/mapping_robot/mapping_robot/profile.py', 'src/mapping_robot/mapping_robot/preflight.py',
    'src/mapping_robot/launch/robot.launch.py', 'src/mapping_semantic/package.xml',
    'src/mapping_semantic/setup.py', 'src/mapping_semantic/mapping_semantic/node.py',
    'src/mapping_semantic/mapping_semantic/detector_node.py',
)


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def read_text(path):
    try:
        return Path(path).read_text().strip()
    except (OSError, UnicodeError):
        return None


def command(arguments, cwd=None):
    result = subprocess.run(arguments, cwd=cwd, capture_output=True, text=True, timeout=20)
    if result.returncode:
        raise ValueError(result.stderr.strip() or '명령 실패: ' + arguments[0])
    return result.stdout.strip()


def save_json(path, value):
    """검사 실패 결과도 보존하되 기존 증거를 덮어쓰지 않는다."""
    with Path(path).open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def parse_os_release(text):
    result = {}
    for line in (text or '').splitlines():
        if '=' in line:
            key, value = line.split('=', 1)
            result[key] = value.strip('"')
    return result


def environment_issues(os_release, architecture, python_version, ros_distro, missing):
    issues = []
    if os_release.get('ID') != 'ubuntu' or os_release.get('VERSION_ID') != '22.04':
        issues.append('기준 환경은 Ubuntu 22.04입니다. OS/JetPack을 자동 변경하지 않습니다')
    if architecture not in ('aarch64', 'x86_64'):
        issues.append('검증 대상 아키텍처는 aarch64/x86_64입니다')
    if tuple(python_version[:2]) != (3, 10):
        issues.append('ROS 2 Humble의 시스템 Python 3.10을 사용하세요')
    if ros_distro != 'humble':
        issues.append('ROS_DISTRO=humble 환경을 source해야 합니다')
    if missing:
        issues.append('필수 ROS 패키지 누락: ' + ', '.join(missing))
    return issues


def host_snapshot():
    release = parse_os_release(read_text('/etc/os-release'))
    packages = {}
    for name in ('nvidia-l4t-core', 'ros-humble-slam-toolbox', 'ros-humble-cv-bridge',
                 'python3-opencv', 'python3-numpy'):
        try:
            packages[name] = command(['dpkg-query', '-W', '-f=${Version}', name])
        except (OSError, ValueError):
            packages[name] = None
    return {
        'schema_version': 1, 'captured_at_utc': datetime.now(timezone.utc).isoformat(),
        'validation_context': 'host_only; not robot acceptance',
        'os': release, 'architecture': platform.machine(), 'kernel': platform.release(),
        'python': platform.python_version(), 'ROS_DISTRO': os.environ.get('ROS_DISTRO'),
        'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0'),
        'l4t_release': read_text('/etc/nv_tegra_release'),
        'device_model': (read_text('/proc/device-tree/model') or '').strip('\x00') or None,
        'packages': packages,
        'disk_free_bytes': shutil.disk_usage(Path.cwd()).free,
        'missing_hardware_evidence': ['센서/드라이버 revision', '시계 동기화 오차',
                                      '장착·RGB-depth 정렬 보정', '전원 모드·물리 정지 검증'],
    }


def check_environment():
    missing = []
    try:
        from ament_index_python.packages import get_package_share_directory
        for name in ROS_PACKAGES:
            try:
                get_package_share_directory(name)
            except LookupError:
                missing.append(name)
    except ImportError:
        missing = list(ROS_PACKAGES)
    host = host_snapshot()
    issues = environment_issues(host['os'], host['architecture'], sys.version_info,
                                host['ROS_DISTRO'], missing)
    host.update({'environment_valid': not issues, 'issues': issues,
                 'missing_ros_packages': missing,
                 'limitations': '환경 확인만 수행; ARM 빌드·센서·모델 실행·실물 성능을 증명하지 않음'})
    return host


def bundle_document(name, data, included, commit):
    """제외한 개발 패키지의 상대 문서 링크는 해당 소스 커밋 URL로 연결한다."""
    if not name.endswith('.md'):
        return data
    def replace(match):
        label, href = match.groups()
        if href.startswith(('#', '/')) or ':' in href:
            return match.group(0)
        path, separator, fragment = href.partition('#')
        target = posixpath.normpath(posixpath.join(posixpath.dirname(name), path))
        if target in included:
            return match.group(0)
        url = ('https://github.com/reconmap-lab/prometheus-mapping/blob/' + commit + '/'
               + quote(target, safe='/') + (separator + fragment if separator else ''))
        return '[' + label + '](' + url + ')'
    return re.sub(r'(?<!!)\[([^\]]+)\]\(([^\s()]+)\)', replace, data.decode('utf-8')).encode('utf-8')


def bundle_workspace(repo, output):
    """Git 커밋의 소스만 묶는다. x86 install/ 및 외부 모델은 포함하지 않는다."""
    repo, output = Path(repo).resolve(), Path(output).resolve()
    if output.exists():
        raise ValueError('bundle 출력 파일은 새 경로여야 합니다')
    commit = command(['git', 'rev-parse', 'HEAD'], repo)
    if command(['git', 'status', '--porcelain', '--untracked-files=normal'], repo):
        raise ValueError('검토·커밋한 clean checkout에서만 bundle을 생성합니다')
    paths = command(['git', 'ls-tree', '-rz', '--name-only', commit, '--', 'src/mapping_robot', 'src/mapping_semantic',
                     'scripts/robot_workspace.py', 'scripts/robot_recording.py', 'docs'], repo)
    names = sorted(name for name in paths.split('\0') if name)
    if not set(BUNDLE_REQUIRED_FILES).issubset(names):
        raise ValueError('실물 패키지 필수 파일이 커밋에 없습니다')
    if not all((repo / name).is_file() and not (repo / name).is_symlink() for name in names):
        raise ValueError('bundle에는 일반 파일만 허용합니다')
    source_files = {name: subprocess.run(['git', 'show', commit + ':' + name], cwd=repo,
                    check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=20).stdout
                    for name in names}
    files = {name: bundle_document(name, data, set(names), commit)
             for name, data in source_files.items()}
    manifest = {
        'schema_version': 1, 'git_commit': commit,
        'runtime_packages': list(RUNTIME_PACKAGES),
        'target': 'Ubuntu 22.04 / ROS 2 Humble; build on target architecture',
        'hardware_tested': False, 'file_sha256': {name: sha256(data) for name, data in files.items()},
        'source_file_sha256': {name: sha256(data) for name, data in source_files.items()},
        'documentation_policy': '제외한 개발 파일의 상대 Markdown 링크만 소스 커밋 URL로 변경; 코드 바이트 보존',
        'external_requirements': ['ROS 2 Humble', 'ROS·Python 의존 패키지', '실물 드라이버·보정 파일',
                                  'optional NanoDet ONNX with expected SHA-256'],
    }
    prefix = 'prometheus-robot-' + commit[:12]
    # xモードで既存成果物を保護。mtime固定で同一ソースの内容を比較しやすくする。
    with output.open('xb') as stream, tarfile.open(fileobj=stream, mode='w:gz') as archive:
        for name, data in {**files, 'bundle_manifest.json':
                           (json.dumps(manifest, ensure_ascii=False, indent=2) + '\n').encode()}.items():
            info = tarfile.TarInfo(prefix + '/' + name)
            info.size, info.mtime, info.mode = len(data), 0, 0o644
            archive.addfile(info, io.BytesIO(data))
    return {'bundle': str(output), 'sha256': sha256(output.read_bytes()), 'git_commit': commit,
            'file_count': len(files), 'runtime_packages': list(RUNTIME_PACKAGES)}


def verify_bundle(root):
    root = Path(root).resolve()
    manifest = json.loads((root / 'bundle_manifest.json').read_text())
    if manifest.get('schema_version') != 1 or manifest.get('runtime_packages') != list(RUNTIME_PACKAGES):
        raise ValueError('bundle manifest 계약 불일치')
    issues = []
    hashes = manifest.get('file_sha256')
    if not isinstance(hashes, dict) or not set(BUNDLE_REQUIRED_FILES).issubset(hashes):
        return {'bundle_valid': False, 'issues': ['필수 런타임 파일의 manifest 항목이 없습니다']}
    actual = {str(path.relative_to(root)) for path in root.rglob('*')
              if (path.is_file() or path.is_symlink()) and path.name != 'bundle_manifest.json'}
    if actual != set(hashes):
        issues.append('manifest와 추출 파일 목록이 다릅니다: 누락 또는 추가 파일')
    for name, expected in hashes.items():
        path = root / name
        if Path(name).is_absolute() or '..' in Path(name).parts or path.is_symlink() or not path.resolve().is_relative_to(root):
            issues.append('유효하지 않은 bundle 경로: ' + name)
        elif not path.is_file() or sha256(path.read_bytes()) != expected:
            issues.append('파일 누락/해시 불일치: ' + name)
    return {'bundle_valid': not issues, 'git_commit': manifest['git_commit'], 'issues': issues,
            'limitations': '내용 무결성 검사이며 배포자 서명·실물 실행 검증은 아님'}


def resource_sample():
    temperatures = {}
    for zone in sorted(Path('/sys/class/thermal').glob('thermal_zone*')):
        value = read_text(zone / 'temp')
        if value is not None:
            try:
                temperatures[read_text(zone / 'type') or zone.name] = float(value) / 1000
            except ValueError:
                pass
    return {'utc': datetime.now(timezone.utc).isoformat(), 'monotonic_ns': time.monotonic_ns(),
            'load_average': list(os.getloadavg()), 'proc_stat': read_text('/proc/stat'),
            'meminfo': read_text('/proc/meminfo'), 'temperature_c': temperatures}


def sample_resources(output, duration, interval):
    if not 0 < duration <= 86400 or not 0 < interval <= duration:
        raise ValueError('0 < interval <= duration <= 86400초여야 합니다')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    save_json(output / 'host.json', host_snapshot())
    stop = time.monotonic() + duration
    samples = 0
    with (output / 'resources.jsonl').open('x') as stream:
        while True:
            stream.write(json.dumps(resource_sample(), ensure_ascii=False, allow_nan=False) + '\n')
            stream.flush()
            samples += 1
            remaining = stop - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(interval, remaining))
    result = {'status': 'completed', 'sample_count': samples,
              'limitations': 'OS 원자료 기록. GPU·전력·추론 지연은 별도 계측 필요; 합격 판정 없음'}
    save_json(output / 'summary.json', result)
    return result


def positive(value):
    result = float(value)
    if not 0 < result <= 86400:
        raise argparse.ArgumentTypeError('유한한 0 초과 86400 이하 값이 필요합니다')
    return result


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    for name in ('snapshot', 'check-env'):
        sub.add_parser(name).add_argument('--output', required=True, type=Path)
    bundle = sub.add_parser('bundle')
    bundle.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[1])
    bundle.add_argument('--output', type=Path, required=True)
    sub.add_parser('verify-bundle').add_argument('root', type=Path)
    sample = sub.add_parser('sample-resources')
    sample.add_argument('--output', type=Path, required=True)
    sample.add_argument('--duration', type=positive, default=60)
    sample.add_argument('--interval', type=positive, default=1)
    options = parser.parse_args(args)
    try:
        if options.action == 'bundle':
            result = bundle_workspace(options.repo, options.output)
        elif options.action == 'verify-bundle':
            result = verify_bundle(options.root)
        elif options.action == 'sample-resources':
            result = sample_resources(options.output, options.duration, options.interval)
        else:
            result = check_environment() if options.action == 'check-env' else host_snapshot()
            save_json(options.output, result)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 1 if result.get('issues') else 0
    except (OSError, ValueError, KeyError, subprocess.SubprocessError) as exc:
        print('실패: ' + str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
