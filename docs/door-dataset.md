# 문짝·출입구 자료 계약과 현재 인벤토리

2026-11-03 [후속 계획](door-next-stage-plan.md)의 라벨·분할 준비를 구현했다.
설계는 [ADR-0011](adr/0011-door-dataset-contract.md)을 따른다.
현재는 자료 점검과 이중 라벨 **초안**이며 학습·독립 시험 준비 완료 자료가 아니다.

## 라벨 계약

[door_dataset.py](../src/mapping_semantic/mapping_semantic/door_dataset.py)의
`validate_dataset(document)`는 schema_version=1 manifest의 독립 사본을 반환한다.
파일 I/O 없이 형식·수치·중복·선언된 분할을 검증하며 원본 객체를 수정하지 않는다.

| 필드 | 의미 |
|---|---|
| dataset_id | 자료 버전 식별 |
| frames[].id | manifest 안의 고유 프레임 ID |
| environment | gazebo / real_rgb / real_rgbd |
| group_id / split | 문·장소·촬영 세션 그룹과 development/train/validation/test/unassigned |
| rgb | 원본 경로·SHA-256·width/height |
| capabilities | depth / camera_info / tf의 제공 여부 |
| review_status | draft / reviewed. 형식 선언이며 검증기가 검수 진위를 증명하지 않음 |
| doors[].id | 해당 프레임 안의 문 ID. 전 프레임의 물리적 추적 ID를 뜻하지 않음 |
| state | open / closed / unknown |
| leaf_bbox / doorway_bbox | 보이는 문짝과 고정 출입구의 독립 xyxy 라벨 |
| leaf_visibility / doorway_visibility | visible / occluded / unknown |

문짝 bbox는 보이는 문짝 부분을, 출입구 bbox는 문틀을 포함한 출입구 영역을 나타낸다.
가려진 전체 문짝을 임의로 복원하지 않는다. 양개문은 하나의 출입구 ID로 관리하고
보이는 문짝 부분들을 감싸는 박스로 기록한다. 대상이 완전히 가려지거나 불명인 경우 bbox는 null이다.
visible에는 유한한 영상 내부 양수 면적 bbox가 필요하다.
draft의 빠진 문 필드는 unknown/null로 채우며 reviewed는 모든 문 필드를 명시해야 한다.

같은 RGB SHA-256의 중복 프레임을 거부한다. 같은 group_id는 미할당을 포함해 한 split에만 존재해야 하며
Gazebo/실제 자료를 같은 그룹으로 섞지 않는다. real_rgb의 depth=true와 real_rgbd의 depth=false를 거부한다.
CameraInfo·TF는 별도로 실제 제공 여부를 선언한다. RGB-D에 보정이 없어도 인벤토리에 등록할 수 있지만,
그 자료로 깊이 투영이 가능하다는 뜻은 아니다.
실제 장소가 서로 같은지와 그룹이 정확한지는 사람이 확인해야 한다.
reviewed 상태만으로 학습 가능하다고 판단하지 말고, 학습 대상의 bbox·가시성·정답 완전성을 확인해야 한다.

## 기본 인벤토리 — 86프레임

[intake 도구](../scripts/door_probe/dataset_intake.py)는 기존 사진과 라벨을 읽기만 하며 원본을 복사하지 않는다.
RGB/depth/기존 라벨의 해시와 크기·방향을 확인한 뒤 로컬 초안을 만든다.

```bash
/usr/bin/python3 scripts/door_probe/dataset_intake.py \
  --captures experiments/door_probe/run-01 \
  --recon-root /home/blind/cv-middle/CV-VLM/recon-report \
  --output experiments/door_probe/dataset-intake-new
```

최종 출력은 로컬 `experiments/door_probe/dataset-intake-03/dataset.json`이며 Git 제외다.
[추적 요약](data/door-dataset-intake-summary.json)에 결과 해시·도구 해시·원본 manifest 해시를 보존했다.

| 자료 | 원본 프레임 | 중복 제거 후 | 이중 bbox 검수 완료 | 분할 |
|---|---:|---:|---:|---|
| Gazebo RGB-D | 108 | 36 | 0 | 단일 geometry development 그룹 |
| 기존 실제 RGB 사진 | 50 | 50 | 0 | source collection별 unassigned |
| 실제 RGB-D | 0 | 0 | 0 | 미확인 |

Gazebo는 동일 RGB 72개를 제외했다. 같은 문·재질·공간을 여러 거리·각도로 본 자료이므로
프레임 무작위 분할로 독립 시험이라고 부르지 않는다.
실제 자료는 기존 recon-report의 DeepDoors2·Open Images 사진이다. 기존 단일 door 라벨은
사용자 검수본이지만 문짝/출입구 bbox 의미가 분리되지 않았다. 원본 검수 이력과 파일 해시는
legacy_labels로 보존하고 **새 두 bbox는 null, 새 검수 상태는 draft**로 남겼다.
장소·촬영 세션을 확인하기 전에는 collection 그룹을 잘게 나누거나 학습/시험으로 배정하지 않는다.
실제 자료에는 depth·CameraInfo·TF가 없어 깊이 상태·미터 단위 지도 위치를 평가할 수 없다.
능력 표시는 기록이 존재한다는 뜻이며 보정 정확도를 검증했다는 뜻은 아니다.

## 확장 검수 묶음 — 95프레임

[실패·시점 비교 검수 묶음](door-label-review.md#실패보조-시점을-포함한-95장-검수-묶음)에
기존 86장과 실제 Gazebo RGB 9장을 함께 준비했다.
로컬 `experiments/door_probe/review-packet-02/dataset.json`·`review.html`을 사용하며
[추적 요약](data/door-review-packet-summary.json)에 원문·도구·결과 해시를 보존했다.

| 종류 | 프레임 수 | depth 기록 | 분할 |
|---|---:|---|---|
| 기존 Gazebo RGB-D | 36 | 있음 | 파생 장면 전체와 같은 development 그룹 |
| 새 Gazebo RGB | 9 | 없음 | 같은 development 그룹, 실패 사례 포함 |
| 기존 실제 RGB | 50 | 없음 | 기존 collection별 unassigned |
| 실제 RGB-D | 0 | 없음 | 2026-11-03 사용자 현재 자료 없음 확인 |

새 예측 박스는 읽기 전용 참고이며 두 정답 bbox는 null/draft다. 현재 이중 bbox 검수 완료는 0장이다.
이미 개발에 사용한 장면과 파생 장면을 한 그룹으로 묶어 독립 시험셋으로 재명명하지 않는다.
검수 후에도 실제 장소·세션 그룹 확인과 학습/검증/시험 분할은 별도로 필요하다.

## 기존 실제 사진 C 결과의 참고 감사

[real_reference.py](../scripts/door_probe/real_reference.py)는 외부 프로젝트의 2026-11-01 완료 원문을 재감사한다.
모델을 다시 실행하지 않는다. 사진·라벨·비교 원문의 해시, 입력 해시와 프레임 대응을 확인한다.
과거 코드 커밋은 `95fe4a679315680321e7b4eceaeae37cd5659ceb`다.
최종 참고 감사 원문은 로컬 `experiments/door_probe/real-reference-03.json`,
[추적 요약](data/door-real-reference-summary.json)에 해당 해시·기존 모델/설정 해시와 프레임별 결과를 보존했다.

| 원래 평가 그룹 | 전체 사진 | 문 개수 평가 대상 | 문 개수 일치 | 기존 기준 제외 |
|---|---:|---:|---:|---:|
| 실내 | 41 | 35 | 20/35 | 6 |
| 실외 보조 | 9 | 9 | 6/9 | 0 |

평가 대상 44프레임의 C 실행 오류는 0이었다. 실행 오류가 있으면 개수 일치율 분모에서 빼지 않는다.
개수가 같아도 정확한 문 bbox를 검출했다는 뜻은 아니다.
C의 상태는 unknown이며 이 표는 문짝/문틀 bbox·깊이 상태·현재 지도 파이프라인의 실제 성능이 아니다.
이전에 사용한 자료이므로 새 독립 평가라고 부르지 않는다. Gazebo의 상태 정확도 분모와 합산하지 않는다.

```bash
/usr/bin/python3 scripts/door_probe/real_reference.py \
  --recon-root /home/blind/cv-middle/CV-VLM/recon-report \
  --output experiments/door_probe/real-reference-new.json
```

## 라벨 작성·검수 도구

[오프라인 검수 화면](door-label-review.md)으로 문짝·출입구 박스를 각각 작성하고,
검수 완료를 명시한 프레임만 검수자·원문 해시와 함께 새 manifest로 저장할 수 있다.
도구 구현과 실제 사람의 라벨 검수 완료는 구분한다. 현재 검수 완료 수는 여전히 0이다.

## 검수 이후 검출 진단

[원 검출의 문짝·출입구 기준별 평가](door-label-evaluation.md)를 준비했다.
reviewed·가시성·환경·분할을 명시하고 오류·누락 분모를 보존한다.
현재 유효 채점 프레임은 0이며 실제 성능을 계산한 결과가 아니다.

## 검증과 다음 입력

데이터 계약 검사 39개와 참고 감사 검사 3개를 통과했다. 실제 86프레임 인벤토리 생성과
기존 실제 50장 원문 감사를 실행했고, 사진·라벨·원문 변경은 하지 않았다.
학습 모델·새 가중치·추론 실행은 이번 작업에서 추가하지 않았다.

다음 학습/평가에는 문짝·출입구 이중 라벨 검수와 실제 장면 그룹 확인이 필요하다.
실제 깊이 상태·지도 검증에는 RGB-D/rosbag 경로, 정렬·CameraInfo·TF, 상태·위치 정답이 필요하다.
현재 그 실제 RGB-D 입력은 확보하지 못했으며 전체 후속 목표는 아직 완료되지 않았다.
