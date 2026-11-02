"""Gazebo RGB 캡처를 recon-report C로 평가하고 원문·기하 기준 비교를 보존한다."""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

from PIL import Image, ImageDraw


IOU_THRESHOLD = 0.5
STATES = {'closed': 0, 'half': 45, 'open': 90, 'negative': None}


def sha256(path):
    checksum = hashlib.sha256()
    with Path(path).open('rb') as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            checksum.update(chunk)
    return checksum.hexdigest()


def write_json(path, document):
    Path(path).write_text(json.dumps(document, ensure_ascii=False, indent=2,
                                   allow_nan=False) + '\n', encoding='utf-8')


def read_json(path):
    def reject_constant(value):
        raise ValueError(f'유한하지 않은 JSON 값: {value}')
    return json.loads(Path(path).read_text(encoding='utf-8'), parse_constant=reject_constant)


def valid_box(box, width=None, height=None):
    if (not isinstance(box, list) or len(box) != 4
            or any(isinstance(value, bool) or not isinstance(value, (int, float))
                   or not math.isfinite(value) for value in box)
            or min(box) < 0 or box[0] >= box[2] or box[1] >= box[3]):
        raise ValueError('bbox는 유한한 비음수 xyxy이며 면적이 양수여야 합니다')
    if width is not None and (box[2] > width or box[3] > height):
        raise ValueError('bbox가 원본 영상 경계를 벗어났습니다')
    return [float(value) for value in box]


def iou(first, second):
    first, second = valid_box(first), valid_box(second)
    intersection = max(0, min(first[2], second[2]) - max(first[0], second[0])) * max(
        0, min(first[3], second[3]) - max(first[1], second[1]))
    area_a = (first[2] - first[0]) * (first[3] - first[1])
    area_b = (second[2] - second[0]) * (second[3] - second[1])
    return intersection / (area_a + area_b - intersection)


def contained_file(root, relative):
    if not isinstance(relative, str) or Path(relative).is_absolute():
        raise ValueError('캡처 파일은 상대 경로여야 합니다')
    path = (root / relative).resolve()
    if not path.is_relative_to(root) or not path.is_file():
        raise ValueError(f'캡처 디렉터리 안의 파일이 아닙니다: {relative}')
    return path


def load_captures(root):
    """부분 실행, 빠진 파일, 중복 프레임을 추론 전에 거부한다."""
    root = Path(root).resolve()
    manifest = read_json(root / 'captures.json')
    if (manifest.get('schema_version') != 1 or manifest.get('environment') != 'gazebo'
            or manifest.get('status') != 'complete'):
        raise ValueError('완료된 schema 1 Gazebo 캡처만 평가합니다')
    count = manifest.get('frames_per_condition')
    if isinstance(count, bool) or not isinstance(count, int) or count < 1:
        raise ValueError('frames_per_condition은 양의 정수여야 합니다')
    conditions = manifest.get('conditions')
    if not isinstance(conditions, list) or not conditions:
        raise ValueError('평가할 조건이 없습니다')
    if manifest.get('total_conditions') != len(conditions):
        raise ValueError('예정 조건 수와 완료 조건 수가 다릅니다')
    seen_ids, seen_files, rows = set(), set(), []
    for condition in conditions:
        identifier, state = condition.get('id'), condition.get('state')
        if (not isinstance(identifier, str) or not re.fullmatch(r'[A-Za-z0-9_-]+', identifier)
                or identifier in seen_ids or state not in STATES):
            raise ValueError('조건 ID 또는 문 상태가 유효하지 않습니다')
        seen_ids.add(identifier)
        if condition.get('door_angle_deg') != STATES[state]:
            raise ValueError('문 상태와 배치 각도가 다릅니다')
        for name in ('distance_m', 'view_angle_deg'):
            value = condition.get(name)
            if (isinstance(value, bool) or not isinstance(value, (float, int))
                    or not math.isfinite(value) or (name == 'distance_m' and value <= 0)):
                raise ValueError(f'유효하지 않은 조건 값: {name}')
        frames = condition.get('frames')
        if not isinstance(frames, list) or len(frames) != count:
            raise ValueError(f'{identifier}: 프레임 수가 계획과 다릅니다')
        for index, frame in enumerate(frames):
            rgb = contained_file(root, frame.get('rgb'))
            metadata_path = contained_file(root, frame.get('metadata'))
            for path in (rgb, metadata_path):
                if path in seen_files:
                    raise ValueError('같은 캡처 파일을 중복 평가할 수 없습니다')
                seen_files.add(path)
            metadata = read_json(metadata_path)
            with Image.open(rgb) as image:
                if image.format != 'PNG' or image.getexif().get(274, 1) != 1:
                    raise ValueError('좌표 보존을 위해 회전 없는 원본 PNG 캡처가 필요합니다')
                width, height = image.size
                image.load()
            if 'truth_bbox_xyxy' not in metadata:
                raise ValueError('프레임 기하 정답이 없습니다')
            truth = metadata['truth_bbox_xyxy']
            if state == 'negative':
                if truth is not None:
                    raise ValueError('음성 대조군에 문 정답이 있습니다')
            else:
                truth = valid_box(truth, width, height)
            rows.append({
                'frame_id': f'{identifier}_{index:02d}', 'condition_id': identifier,
                'state': state, 'door_angle_deg': condition['door_angle_deg'],
                'distance_m': condition['distance_m'], 'view_angle_deg': condition['view_angle_deg'],
                'rgb': str(rgb), 'metadata': str(metadata_path),
                'rgb_sha256': sha256(rgb), 'metadata_sha256': sha256(metadata_path),
                'width': width, 'height': height, 'truth_bbox_xyxy': truth,
            })
    return manifest, rows


def score_frame(truth, detections, width, height):
    """문틀 기하 정답 하나에 최대 한 검출만 대응시킨다."""
    if not isinstance(detections, list):
        raise ValueError('C 결과에 detections 배열이 없습니다')
    doors = []
    for detection in detections:
        if not isinstance(detection, dict):
            raise ValueError('검출 record가 사전이 아닙니다')
        label = detection.get('class')
        if label not in ('door', 'person') or detection.get('report_class') != label:
            raise ValueError('문·사람 검출 계약이 다릅니다')
        box = valid_box(detection.get('bbox'), width, height)
        confidence = detection.get('confidence')
        if (isinstance(confidence, bool) or not isinstance(confidence, (int, float))
                or not math.isfinite(confidence) or not 0 <= confidence <= 1):
            raise ValueError('검출 confidence가 유효하지 않습니다')
        if label == 'door':
            doors.append({**detection, 'bbox': box})
    if truth is None:
        return {'status': 'true_negative' if not doors else 'false_positive',
                'success': not doors, 'tp': 0, 'fn': 0, 'fp': len(doors),
                'best_iou': None, 'matched_index': None, 'door_detections': doors}
    valid_box(truth, width, height)
    overlaps = [iou(truth, door['bbox']) for door in doors]
    best = max(overlaps, default=0.0)
    matched = overlaps.index(best) if best >= IOU_THRESHOLD else None
    return {'status': 'true_positive' if matched is not None else 'missed',
            'success': matched is not None, 'tp': int(matched is not None),
            'fn': int(matched is None), 'fp': len(doors) - int(matched is not None),
            'best_iou': best, 'matched_index': matched, 'door_detections': doors}


def summarize(rows):
    grouped = {}
    for row in rows:
        identifier = row['condition_id']
        if identifier not in grouped:
            grouped[identifier] = {name: row[name] for name in (
                'condition_id', 'state', 'door_angle_deg', 'distance_m', 'view_angle_deg')}
            grouped[identifier].update(frames=0, successful_frames=0, runtime_errors=0,
                                       tp=0, fn=0, fp_observed=0)
        summary = grouped[identifier]
        summary['frames'] += 1
        summary['successful_frames'] += int(row['success'])
        if row['status'] == 'runtime_error':
            summary['runtime_errors'] += 1
        else:
            summary['tp'] += row['tp']
            summary['fn'] += row['fn']
            summary['fp_observed'] += row['fp']
    for summary in grouped.values():
        summary['success_rate'] = summary['successful_frames'] / summary['frames']
    return list(grouped.values())


def terminate_group(process):
    """이번 호출에서 만든 프로세스 그룹만 단계적으로 종료한다."""
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        try:
            return process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            continue
    return process.communicate(timeout=5)


def run_cli(command, cwd, timeout, directory):
    started = time.monotonic()
    process = subprocess.Popen(command, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               text=True, start_new_session=True)
    timed_out = False
    try:
        try:
            stdout, stderr = process.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            timed_out = True
            stdout, stderr = terminate_group(process)
    except BaseException:
        terminate_group(process)
        raise
    directory.joinpath('stdout.json').write_text(stdout, encoding='utf-8')
    directory.joinpath('stderr.txt').write_text(stderr, encoding='utf-8')
    execution = {'command': command, 'cwd': str(cwd), 'pid': process.pid,
                 'returncode': process.returncode, 'timed_out': timed_out,
                 'wall_seconds': time.monotonic() - started}
    write_json(directory / 'execution.json', execution)
    return execution, stdout


def provenance(recon_root):
    config_path = recon_root / 'experiments/configs/yoloe-door-person.json'
    config = read_json(config_path)
    assets = {}
    for name in ('checkpoint', 'embeddings', 'manifest', 'prompts_config'):
        asset = (config_path.parent / config[name]).resolve()
        assets[name] = {'path': str(asset), 'sha256': sha256(asset)}
    revision = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=recon_root,
                              capture_output=True, text=True, check=True).stdout.strip()
    dirty = subprocess.run(['git', 'status', '--short'], cwd=recon_root,
                           capture_output=True, text=True, check=True).stdout.strip()
    sources = {}
    for relative in ('src/recon_report/run.py', 'src/recon_report/focused.py',
                     'src/recon_report/tasks/door_person.py', 'src/recon_report/grounding/yoloe.py'):
        sources[relative] = sha256(recon_root / relative)
    return {'root': str(recon_root), 'revision': revision, 'git_status_short': dirty,
            'config_path': str(config_path), 'config_sha256': sha256(config_path),
            'config': config, 'assets': assets, 'sources_sha256': sources}


def overlay(row, destination):
    with Image.open(row['rgb']) as source:
        image = source.convert('RGB')
    draw = ImageDraw.Draw(image)
    if row['truth_bbox_xyxy'] is not None:
        draw.rectangle(row['truth_bbox_xyxy'], outline=(0, 255, 0), width=3)
    for index, door in enumerate(row.get('door_detections', [])):
        color = (0, 180, 255) if index == row.get('matched_index') else (255, 70, 70)
        draw.rectangle(door['bbox'], outline=color, width=2)
        x, y = door['bbox'][:2]
        draw.text((x + 3, y + 3), f"door {door['confidence']:.3f}", fill=color)
    draw.rectangle((0, 0, image.width, 29), fill=(0, 0, 0))
    draw.text((5, 3), f"{row['frame_id']} / {row['status']}", fill='white')
    draw.text((5, 16), 'green: geometric GT; blue: matched; red: unmatched', fill='white')
    image.save(destination)


def write_reports(output, document):
    summaries = summarize(document['frames'])
    document['summary'] = summaries
    write_json(output / 'results.json', document)
    if summaries:
        with (output / 'summary.csv').open('w', encoding='utf-8', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(summaries[0]))
            writer.writeheader()
            writer.writerows(summaries)
    lines = [
        '# Gazebo 문 검출 첫 실험', '',
        f"평가 상태: `{document['status']}`. 처리 프레임 {len(document['frames'])}/{document['expected_frames']}.", '',
        'C는 문 검출만 수행하며 상태 출력은 unknown이다. 문 상태 정확도는 측정하지 않는다.', '',
        '정답은 문틀 바깥 경계를 원본 영상으로 투영하고 클립한 기하 bbox다. '
        '가림 여부를 자동 판정하지 않으며 수동 라벨과 동일하다고 가정하지 않는다. '
        'IoU≥0.5인 door 검출 하나를 TP로 대응하고 추가 검출은 FP로 센다. '
        '음성 대조군은 door 검출이 없어야 성공이다.', '',
        '실행 오류도 성공률 분모에 포함한다. FP·FN은 유효 추론에서 관측한 값만 합산하며 '
        '실행 오류의 검출 수를 0으로 채우지 않는다. 동일 정적 장면의 반복 프레임은 '
        '독립적인 환경이 아니다. 실제 촬영 영상 성능과 분리한다.', '',
        '| 조건 | 거리(m) | 관측각(°) | 문 상태 | 성공/전체 | 실행 오류 | 관측 FP |',
        '|---|---:|---:|---|---:|---:|---:|',
    ]
    for row in summaries:
        lines.append(f"| {row['condition_id']} | {row['distance_m']} | {row['view_angle_deg']} | "
                     f"{row['state']} | {row['successful_frames']}/{row['frames']} | "
                     f"{row['runtime_errors']} | {row['fp_observed']} |")
    lines += ['', '원본 stdout·stderr·실행 명령·recon-report 로그는 `raw/`, '
              '기하 정답(초록)·대응 검출(파랑)·미대응 검출(빨강)은 `overlays/`에 보존한다.', '']
    (output / 'report.md').write_text('\n'.join(lines), encoding='utf-8')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--captures', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='새 결과 디렉터리')
    parser.add_argument('--recon-root', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=120, help='프레임당 제한 초')
    args = parser.parse_args(argv)
    try:
        if not math.isfinite(args.timeout) or args.timeout <= 0:
            raise ValueError('timeout은 유한한 양수여야 합니다')
        root, output, recon_root = args.captures.resolve(), args.output.resolve(), args.recon_root.resolve()
        if output.exists():
            raise ValueError('기존 결과 디렉터리는 덮어쓰지 않습니다')
        manifest, frames = load_captures(root)
        python = recon_root / '.venv/bin/python'
        if not python.is_file():
            raise ValueError('recon-report의 기존 .venv/bin/python이 필요합니다')
        identity = provenance(recon_root)
    except (ValueError, OSError, KeyError, TypeError, subprocess.CalledProcessError) as exc:
        parser.error(str(exc))
    output.mkdir(parents=True)
    (output / 'overlays').mkdir()
    document = {'schema_version': 1, 'environment': 'gazebo', 'method': 'C', 'status': 'running',
                'created_at': datetime.now(timezone.utc).isoformat(), 'captures': str(root),
                'captures_sha256': sha256(root / 'captures.json'), 'captures_manifest': manifest,
                'evaluator_sha256': sha256(__file__), 'recon': identity,
                'iou_threshold': IOU_THRESHOLD, 'expected_frames': len(frames), 'frames': []}
    write_reports(output, document)
    try:
        for frame in frames:
            raw = output / 'raw' / frame['frame_id']
            raw.mkdir(parents=True)
            command = ['env', '-u', 'PYTHONPATH', '-u', 'PYTHONHOME', str(python),
                       '-m', 'recon_report.run', '--method', 'C', '--image', frame['rgb'],
                       '--log-dir', str(raw / 'recon'), '--detector-config', identity['config_path']]
            row = dict(frame)
            try:
                if sha256(frame['rgb']) != frame['rgb_sha256']:
                    raise ValueError('추론 전 원본 이미지가 바뀌었습니다')
                execution, stdout = run_cli(command, recon_root, args.timeout, raw)
                row['execution'] = execution
                result = json.loads(stdout)
                if execution['timed_out'] or execution['returncode'] != 0:
                    raise ValueError(f"C 실행 실패: exit={execution['returncode']}, timeout={execution['timed_out']}")
                if (not isinstance(result, dict) or result.get('schema_valid') is not True
                        or result.get('error') is not None):
                    raise ValueError('C 보고서 스키마 검증 실패')
                report = result.get('report', {})
                if (not isinstance(report, dict) or report.get('method') != 'C'
                        or report.get('scope') != 'door-person'):
                    raise ValueError('C 문·사람 보고서가 아닙니다')
                states = report.get('states')
                if (not isinstance(states, list) or any(not isinstance(state, dict)
                        or state.get('state') != 'unknown' for state in states)):
                    raise ValueError('C의 unknown 상태 계약이 바뀌었습니다')
                row.update(score_frame(frame['truth_bbox_xyxy'], result.get('detections'),
                                       frame['width'], frame['height']))
            except (ValueError, OSError, TypeError, KeyError, subprocess.SubprocessError) as exc:
                row.update(status='runtime_error', success=False, tp=None, fp=None, fn=None,
                           best_iou=None, matched_index=None, door_detections=[], error=str(exc))
            row['overlay'] = f"overlays/{frame['frame_id']}.png"
            overlay(row, output / row['overlay'])
            document['frames'].append(row)
            write_reports(output, document)
            print(f"{len(document['frames'])}/{len(frames)} {frame['frame_id']}: {row['status']}", flush=True)
        document['status'] = 'complete'
        document['completed_at'] = datetime.now(timezone.utc).isoformat()
    except BaseException:
        document['status'] = 'interrupted'
        write_reports(output, document)
        raise
    write_reports(output, document)
    return 1 if any(row['status'] == 'runtime_error' for row in document['frames']) else 0


if __name__ == '__main__':
    raise SystemExit(main())
