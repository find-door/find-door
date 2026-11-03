"""별도 가상환경에서 설정으로 선택한 로컬 문 검출기를 유지하는 JSON-lines worker."""

import argparse
from contextlib import redirect_stdout
import hashlib
import json
from pathlib import Path
import sys
import time


def runtime_config(source, directory):
    """외부 설정을 변경하지 않고 자산 절대경로·상주 정책을 명시한다."""
    source, directory = Path(source).resolve(), Path(directory)
    original = source.read_bytes()
    config = json.loads(original)
    config.pop('backend', None)
    for name in ['checkpoint', 'embeddings', 'manifest', 'prompts_config']:
        asset = (source.parent / config[name]).resolve()
        if not asset.is_file():
            raise ValueError(f'로컬 검출 자산 없음: {name}')
        config[name] = str(asset)
    config['release_policy'] = 'keep'
    target = directory / 'detector-config.json'
    target.write_text(json.dumps(config, ensure_ascii=False, indent=2) + '\n')
    return target, hashlib.sha256(original).hexdigest()


def backend_name(source):
    config = json.loads(Path(source).read_text())
    if not isinstance(config, dict) or config.get('backend', 'yoloe') not in ('yoloe', 'yolo11'):
        raise ValueError('검출 backend는 yoloe 또는 yolo11이어야 합니다')
    return config.get('backend', 'yoloe')


def load_detector(args):
    """기본 YOLOE와 명시적으로 선택한 YOLO11만 연결한다."""
    selected = backend_name(args.config)
    if selected == 'yolo11':
        # 직접 실행한 worker의 가상환경에서도 이 저장소 패키지를 찾는다.
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from mapping_semantic.yolo11_doorway import YOLO11DoorwayDetector
        return (YOLO11DoorwayDetector(args.config, args.runtime),
                hashlib.sha256(args.config.read_bytes()).hexdigest(), selected)
    import recon_report.grounding.yoloe as backend
    config, source_hash = runtime_config(args.config, args.runtime)
    original_runtime = backend._runtime
    backend._runtime = lambda ignored: original_runtime(args.runtime / 'ultralytics-config')
    return backend.YOLOEDetector(config), source_hash, selected


def serialize_detection(detection):
    return detection if isinstance(detection, dict) else detection.model_dump(by_alias=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recon-root', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--runtime', type=Path, required=True)
    args = parser.parse_args()
    sys.path.insert(0, str(args.recon_root / 'src'))
    detector = None
    try:
        with redirect_stdout(sys.stderr):
            detector, source_hash, selected_backend = load_detector(args)
        for line in sys.stdin:
            request = None
            start = time.monotonic()
            try:
                request = json.loads(line)
                if type(request.get('id')) is not int or not isinstance(request.get('image_path'), str):
                    raise ValueError('worker 요청 형식 오류')
                if hashlib.sha256(args.config.read_bytes()).hexdigest() != source_hash:
                    raise ValueError('실행 중 원본 검출 설정 변경')
                with redirect_stdout(sys.stderr):
                    detections = detector.detect(Path(request['image_path']).read_bytes())
                reply = dict(id=request['id'], detections=[serialize_detection(d) for d in detections],
                             metadata=dict(backend=selected_backend, source_config_sha256=source_hash, release_policy_override='keep',
                                           cache_override=str(args.runtime / 'ultralytics-config'),
                                           last_run=detector.last_run), inference_wall_sec=time.monotonic()-start)
            except Exception as exc:
                reply = dict(id=request.get('id') if isinstance(request, dict) else None,
                             error=f'{type(exc).__name__}: {exc}')
            print(json.dumps(reply, ensure_ascii=False, allow_nan=False), flush=True)
    finally:
        if detector is not None:
            with redirect_stdout(sys.stderr):
                detector.close()


if __name__ == '__main__':
    main()
