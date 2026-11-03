# 문짝·출입구 개발 학습 입력

2026-11-03 [AI 라벨 진단](door-real-ai-evaluation.md) 이후 반복 문·장소와 출처를 확인하고
학습/검증 입력을 준비했다. **AI 라벨 기반의 소규모 개발 학습용이며 사람 정답·독립 시험셋이 아니다.**
모델 학습·새 가중치 다운로드·모델 교체는 이 단계에서 실행하지 않았다.
이후 [YOLO11 첫 개발 학습](door-yolo11-pilot.md)을 별도 단계에서 실행했다. 기본 모델은 교체하지 않았다.

## 그룹과 분할

DeepDoors2 30장에는 같은 문·복도를 여러 상태와 방향에서 찍은 사진이 반복된다.
종이가 붙은 방문, 작은 유리 현관, 큰 유리 석재문, 옷·꽃병이 있는 복도, 청백색 타일의 격자문,
흰 유리 요소가 있는 복도에서 강한 시각 대응을 확인했다.
서로 다른 문이 같은 집에 속할 가능성도 배제할 수 없어 **DeepDoors2 전체 collection을 한 train 그룹**으로 유지했다.
[시각 대응 후보와 한계](data/door-training-group-audit.json)를 보존하며 소그룹을 독립 장소로 승격하지 않는다.

Open Images 20장은 작성자 프로필과 원 사진 페이지가 모두 다르고, 사진에서 같은 방·문을 확인하지 못했다.
작성자 프로필별로 그룹을 만들고 같은 작성자를 train/validation 양쪽에 넣지 않는다.
작성자가 다르다고 실제 장소·촬영 세션의 독립성이 증명되는 것은 아니다.

검증 그룹은 예측·성능을 입력으로 사용하지 않고 **seed 20261103과 그룹 ID의 SHA-256 순서**로 선택했다.
Open Images 중 두 target이 모두 visible이고 AI 검토 완료인 그룹에서 closed 1그룹·open 2그룹·음성 4그룹을 선택했다.
나머지는 개발 학습 후보이며 실제 출력 시 불확실·target 가시성 불명 프레임을 제외한다.
[고정 분할 계획](data/door-training-split-plan.json)에 전체 50장의 출처와 제안 역할을 기록했다.
후보 역할이 train이어도 제외 조건에 해당하면 이미지·빈 라벨을 생성하지 않는다.

상류 자료의 DeepDoors2 `test`와 Open Images `validation`은 출처 메타데이터로 그대로 남겼다.
이들을 이번 과목의 개발 학습에 사용하면 **상류 공식 test/validation 성능이나 새 독립 시험으로 보고할 수 없다.**
이미 기준선 진단·라벨 작성에서 관찰한 자료이며 검증 양성도 3장뿐이라 모델 일반화·채택을 보장하지 않는다.
별도의 독립 test는 준비되지 않았다. 기존 Gazebo 45장과 사용자 검수 9장은 이번 학습/검증에 넣지 않았다.

## 두 학습 target

문짝과 고정 출입구의 박스 의미가 달라 각각 별도 단일 클래스 입력으로 만들었다.
문짝이 영상에 보이지 않는 열린 문까지 다루려면 문짝 검출만으로 해결되지 않을 수 있으므로,
고정 출입구 후보도 별도로 준비했다. 두 target을 같은 class의 정답으로 섞지 않는다.

| target | YOLO class 0 | train 사진 | train 문 객체 | validation 사진 | validation 문 객체 |
|---|---|---:|---:|---:|---:|
| leaf | door_leaf | 32 | 27 | 7 | 3 |
| doorway | doorway | 32 | 28 | 7 | 3 |

각 train은 양성 25장·음성 7장, validation은 양성 3장·음성 4장이다.
두 validation의 RGB 7장과 출처 그룹은 같다. train은 target별로 일부 사진과 객체 수가 다르다.
전체 실제 사진 50장 중 각 target에서 39장을 사용하고, 프레임 불확실성 9장·target 가시성 불명 2장을 제외했다.
문이 있을 가능성이 남아 있거나 해당 target이 불명인 프레임을 빈 YOLO 라벨로 바꿔 음성으로 학습시키지 않는다.
대신 AI 검토에서 문 없음으로 기록한 실제 음성만 빈 라벨 파일로 내보냈다.
상태 open/closed/unknown은 메타데이터이며 검출 클래스에 나누어 넣지 않는다.

| 공통 검증 사진 (`real_oi_` 생략) | 역할 |
|---|---|
| 28f45b12ee25ae19 | 음성 |
| 32255e4104d8f9b7 | open 양성 |
| 44a1c33f790c5687 | closed 양성 |
| 294885a08c11dfa4 | 음성 |
| e6d27669b2efb650 | open 양성 |
| 05042cfa44abe4f6 | 음성 |
| 1229d5e203956ac6 | 음성 |

## 생성과 원본 보존

[생성 도구](../scripts/door_probe/training_dataset.py)는 원본 AI manifest와 감사한 상류 manifest 해시를 확인한다.
각 사진의 SHA·크기·EXIF를 검사하고 원본 사진에는 쓰지 않는다. 출력 폴더의 images에는 원본으로 향하는 symlink만 만든다.
labels는 class 0과 정규화된 중심 xywh이며 원본 픽셀 bbox에서 계산한다.
YAML에는 train/val과 클래스 이름만 있고 test·download 항목을 넣지 않는다.
형식은 [Ultralytics 공식 detection dataset 안내](https://docs.ultralytics.com/datasets/detect/)를 따른다.

```bash
/usr/bin/python3 scripts/door_probe/training_dataset.py \
  --manifest experiments/door_probe/ai-labels-01/dataset-ai-reviewed.json \
  --source-manifest /home/blind/cv-middle/CV-VLM/recon-report/experiments/configs/validation-v1-reviewed.json \
  --target leaf --output experiments/door_probe/training-data-new/leaf
```

출입구 입력은 `--target doorway`와 별도 출력 경로를 지정한다. 기존 출력 폴더는 덮어쓰지 않는다.
이번 로컬 출력은 `experiments/door_probe/training-data-01/{leaf,doorway}/`다.
각 폴더에 `dataset.yaml`, 원본 provenance를 보존한 `manifest.json`, `split-plan.json`,
제외 이유·라벨 해시·분모를 포함한 `export-report.json`과 images/labels가 있다.
[추적 요약](data/door-training-data-summary.json)에 원본·분할·코드·출력 해시를 기록했다.
이미지 symlink와 원본 사진은 Git에 넣지 않는다.

## 검증과 다음 실행

출처 그룹 누출 방지·결정적 분할·불명 음성화 방지·좌표 복원·상대 경로·원본 불변·해시·덮어쓰기 보호 등
관련 11검사가 통과했다. 전체 오프라인 799검사 통과·ROS 전용 5검사 건너뜀이다.
두 실제 출력에서도 이미지/라벨 해시·원본 bbox 복원과 train/validation 그룹·RGB 분리를 확인했다.
제품 ROS 패키지는 변경하지 않아 패키지 빌드·시뮬레이션은 반복하지 않았다.

다음에는 이 고정 입력으로 YOLO11 소규모 학습의 실행 가능성·메모리·검출 변화를 확인한다.
같은 validation RGB와 target 의미로 기존 C와 후보를 비교하고, 유효 분모·오류·누락을 유지한다.
개발 검증 7장만의 개선으로 모델을 기본 채택하거나 처음 보는 열린 문의 해결·실물 지도 성능을 주장하지 않는다.
기존 Gazebo 개발 회귀와 별도 독립 자료 평가가 추가로 필요하다.
