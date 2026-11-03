"""출처 그룹 분리·불명 음성화 방지·정규화 좌표·원본 불변을 확인한다."""
from copy import deepcopy
import json
from pathlib import Path

from PIL import Image
import pytest
import yaml

from dataset_intake import digest
from label_review import read_manifest
from training_dataset import export_dataset, make_plan, select_target


def fixture(root):
    frames, sources = [], []
    specifications = [('deep0','closed'),('deep1','open'),('closed0','closed'),('closed1','closed'),
        ('open0','open'),('open1','open')] + [(f'negative{i}',None) for i in range(5)] + [
        ('uncertain','closed'),('leaf_unknown','open'),('doorway_unknown','closed')]
    for index, (identifier, state) in enumerate(specifications):
        image = root / (identifier+'.png')
        Image.new('RGB', (100,80), (index*13,20,30)).save(image)
        door = dict(id='door1', state=state, leaf_bbox=[10,10,30,60], doorway_bbox=[5,5,50,70],
                    leaf_visibility='visible', doorway_visibility='visible')
        if identifier.endswith('_unknown'):
            target = identifier.split('_')[0]
            door[target+'_bbox'] = None
            door[target+'_visibility'] = 'unknown'
        frame = dict(id=identifier, environment='real_rgb', group_id='original_collection', split='unassigned',
            rgb=dict(path=image.name, sha256=digest(image), width=100,height=80),
            capabilities=dict(depth=False,camera_info=False,tf=False),
            review_status='draft' if identifier=='uncertain' else 'reviewed',
            annotation_uncertain=identifier=='uncertain',
            annotation_review=dict(method='ai_visual', human_reviewed=False), doors=[] if state is None else [door])
        frames.append(frame)
        deep = identifier.startswith('deep')
        sources.append(dict(frame_id=identifier, sha256=digest(image), dataset='DeepDoors2' if deep else 'Open Images V7',
            split='test' if deep else 'validation',source=dict(AuthorProfileURL=f'https://www.flickr.com/people/{identifier}/')))
    dataset = dict(schema_version=1, dataset_id='fixture', frames=frames)
    source = dict(frames=sources)
    paths = [root/'dataset.json',root/'source.json',root/'reference.json']
    paths[0].write_text(json.dumps(dataset));paths[1].write_text(json.dumps(source))
    paths[2].write_text(json.dumps(dict(manifest_sha256=digest(paths[1]))))
    return dataset, source, paths


def test_plan_is_deterministic_and_keeps_collection_together(tmp_path):
    dataset, source, _ = fixture(tmp_path)
    plan = make_plan(dataset,source)
    reversed_dataset = deepcopy(dataset);reversed_dataset['frames'].reverse()
    assert plan['validation_groups'] == make_plan(reversed_dataset,source)['validation_groups']
    deep = [r for r in plan['frames'] if r['id'].startswith('deep')]
    assert len({r['group_id'] for r in deep})==1
    assert {r['proposed_split'] for r in deep}=={'train'}
    assert {r['source_split'] for r in deep}=={'test'}
    assert len(plan['validation_groups'])==7


def test_same_author_cannot_cross_splits(tmp_path):
    dataset, source, _ = fixture(tmp_path)
    profiles = [s for s in source['frames'] if s['frame_id'].startswith('closed')]
    # 작성자 이름의 대소문자·끝 slash 변형도 보수적으로 같은 그룹에 묶는다.
    profiles[1]['source']['AuthorProfileURL'] = profiles[0]['source']['AuthorProfileURL'].replace('closed0','CLOSED0').rstrip('/')
    rows = [r for r in make_plan(dataset,source)['frames'] if r['id'].startswith('closed')]
    assert len({r['group_id'] for r in rows})==1
    assert len({r['proposed_split'] for r in rows})==1


def test_unknown_target_is_excluded_instead_of_negative(tmp_path):
    dataset, source, _ = fixture(tmp_path);plan=make_plan(dataset,source)
    leaf, exclusions = select_target(dataset,plan,'leaf')
    ids={f['id'] for f in leaf['frames']}
    assert 'leaf_unknown' not in ids and 'uncertain' not in ids and 'doorway_unknown' in ids
    assert {r['id'] for r in exclusions}=={'leaf_unknown','uncertain'}
    doorway,_=select_target(dataset,plan,'doorway')
    assert 'leaf_unknown' in {f['id']for f in doorway['frames']}
    assert 'doorway_unknown' not in {f['id']for f in doorway['frames']}


def test_export_roundtrip_relative_paths_negatives_and_immutable_source(tmp_path):
    dataset, _, paths=fixture(tmp_path)
    before={p:p.read_bytes()for p in tmp_path.iterdir() if p.is_file()}
    output=tmp_path/'export';result=export_dataset(*paths,'leaf',output)
    exported,_=read_manifest(output/'manifest.json')
    assert all(Path(f['rgb']['path']).is_absolute()for f in exported['frames'])
    by_id={f['id']:f for f in exported['frames']}
    for row in result['frames']:
        path=Path(row['image']);label=Path(row['label'])
        assert path.is_symlink() and digest(path)==row['rgb_sha256']
        fields=label.read_text().splitlines()
        assert len(fields)==row['objects']
        for line,door in zip(fields,by_id[row['id']]['doors']):
            cls,x,y,w,h=map(float,line.split())
            assert cls==0 and [(x-w/2)*100,(y-h/2)*80,(x+w/2)*100,(y+h/2)*80]==pytest.approx(door['leaf_bbox'])
        if not row['objects']:assert label.read_bytes()==b''
    assert all(p.read_bytes()==data for p,data in before.items())
    config=yaml.safe_load((output/'dataset.yaml').read_text())
    assert config['names']=={0:'door_leaf'} and 'test'not in config and 'download'not in config
    assert sum(result['split_counts'].values())==12


def test_existing_output_is_not_modified(tmp_path):
    _,_,paths=fixture(tmp_path);output=tmp_path/'export';output.mkdir();marker=output/'keep';marker.write_text('original')
    with pytest.raises(FileExistsError):export_dataset(*paths,'leaf',output)
    assert marker.read_text()=='original'


def test_changed_source_hash_rejected_before_output(tmp_path):
    _,_,paths=fixture(tmp_path);paths[1].write_text('{}');output=tmp_path/'export'
    with pytest.raises(ValueError,match='해시'):export_dataset(*paths,'leaf',output)
    assert not output.exists()


def test_changed_rgb_rejected_before_output(tmp_path):
    _,_,paths=fixture(tmp_path);(tmp_path/'deep0.png').write_bytes(b'changed');output=tmp_path/'export'
    with pytest.raises(ValueError,match='해시'):export_dataset(*paths,'leaf',output)
    assert not output.exists()


@pytest.mark.parametrize('field,value',[('annotation_uncertain',True),('annotation_review',dict(method='ai_visual',human_reviewed=True))])
def test_inconsistent_review_declarations_rejected(tmp_path,field,value):
    dataset,source,_=fixture(tmp_path);dataset['frames'][0][field]=value
    with pytest.raises(ValueError,match='AI 검토'):make_plan(dataset,source)


def test_plan_cannot_put_same_group_into_both_splits(tmp_path):
    dataset,source,_=fixture(tmp_path);plan=make_plan(dataset,source)
    plan['frames'][0]['proposed_split']='validation'
    with pytest.raises(ValueError,match='분할 누출'):select_target(dataset,plan,'leaf')


def test_duplicate_source_or_insufficient_validation_groups_rejected(tmp_path):
    dataset,source,_=fixture(tmp_path);source['frames'].append(deepcopy(source['frames'][0]))
    with pytest.raises(ValueError,match='중복'):make_plan(dataset,source)
    source['frames'].pop()
    for row in source['frames']:
        if row['frame_id']=='open1':row['source']['AuthorProfileURL']='https://www.flickr.com/people/open0/'
    with pytest.raises(ValueError,match='검증 그룹 수 부족'):make_plan(dataset,source)
