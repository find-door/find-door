# ADR-0014: 학습한 YOLO11 출입구 후보의 선택 실행

- **상태:** ✅ 결정 — Gazebo 개발 후보 연결, 기본 모델 채택은 유보
- **날짜:** 2026-11-03
- **관련 단계:** Find Door 실시간 통합·학습 후보 검증

## 배경

[150 epoch 출입구 모델](../door-yolo11-budget.md)은 낮은 로봇 시점의 저장 Gazebo RGB에서
완전 열림 bbox를 회복했지만 실제 AI 개발 검증에서는 악화됐다.
기존 YOLOE 기본값을 유지하면서 새 모델을 실제 RGB-D·상태·지도 경로에서 검증할 수 있어야 한다.
ROS Python에 학습 의존성을 직접 넣거나 외부 recon-report 코드를 수정하지 않는다.

## 선택지와 비교

| 방식 | 장점 | 제약 |
|---|---|---|
| 기존 YOLOE를 곧바로 교체 | 실행 단순 | 실제 검증 악화·개발 선택 편향을 무시 |
| ROS 노드에 YOLO11 직접 적재 | 구현 단순 | ROS와 ML 환경 결합, 기존 동기화 구조 중복 |
| 기존 worker의 설정별 분기 | 기존 원본 RGB-D 묶음·시간·프로세스 구조 유지 | 별도 설정 검증과 회귀 검사 필요 |

## 결정과 이유

ADR-0010의 JSON-lines worker와 `detector_config` 경로를 유지한다.
설정의 backend가 없거나 yoloe이면 기존 YOLOE를 쓰고, backend=yolo11인 별도 설정만 새 로컬 후보를 사용한다.
명시적 yoloe 선택 필드는 외부 YOLOE에 전달할 임시 설정에서 제거하며 원본은 변경하지 않는다.
알 수 없는 backend는 거부한다. 기본 launch·설정·기존 checkpoint는 바꾸지 않는다.

YOLO11 설정은 다음 필드를 모두 요구하고 추가 키를 거부한다.

| 필드 | 계약 |
|---|---|
| schema_version / backend | 정수 1 / yolo11 |
| checkpoint / checkpoint_sha256 | 설정 파일 기준 로컬 .pt 경로·소문자 SHA-256, 실제 바이트 일치 |
| training_target | 현재는 doorway만 허용 |
| ultralytics_version | 실제 설치 버전과 정확히 일치 |
| device | 문자열 0 또는 cpu |
| imgsz | 32~1280의 32배수 정수 |
| confidence / iou | 유한한 [0,1] 점수 / (0,1] NMS 문턱 |
| max_det | 1~300 정수 |

가상환경·모델은 이미 준비된 로컬 파일만 사용하고 자동 설치·다운로드는 끈다.
Ultralytics 설정·캐시는 worker 전용 임시 경로에 두고 원격 integration은 끈다.
모델은 한 번 적재해 유지하며 매 요청에서 설정·checkpoint 식별이 바뀌면 실패한다.
입력은 원본 좌표계의 RGB PNG·EXIF 방향 1·단일 프레임이다. worker에서 별도 회전·자르기를 하지 않는다.

checkpoint class 이름은 {0: doorway}여야 한다. 출력은 source_label=doorway를 남기고
기존 검출 seed 경계인 class=report_class=door로 변환한다. bbox는 원본 픽셀 xyxy다.
잘못된 클래스·유한하지 않은 점수·영상 밖 bbox는 거부하고 빈 성공으로 바꾸지 않는다.
모델/backend/학습 target/버전/checkpoint·입력 해시를 상태 metadata에 보존한다.

RGB-D·CameraInfo·Detection2DArray의 원래 header와 exact 동기화, 관측 시각 TF 조회는 그대로다.
학습한 bbox를 출입구·위치 정답으로 쓰지 않고 기존 RGB-D 문틀·주변 벽 검증을 통과해야 한다.
학습 후보 선택만으로 깊이 상태·지도 효과가 검증됐다고 간주하지 않는다.

## 포기한 것 / 위험

이 설정은 현재 단일 doorway 클래스의 개발 연결이다. 일반 클래스 매핑·실물 모델 채택을 결정하지 않는다.
같은 개발 validation으로 모델과 실험을 선택한 한계, 가구·벽 오검출, 다른 장소의 일반화 위험이 남는다.
첫 추론 적재 지연 중 오래된 RGB-D는 기존 mapper의 지연 제한으로 거부된다.
worker는 소유한 프로세스 그룹만 종료하며 실패 뒤 계속 정상 요청을 보내지 않는다.

## 검증 방법

설정·checkpoint·PNG·클래스·좌표·점수·설치 버전·종료를 모델 없이 단위 검사하고,
기존 YOLOE 응답·설정·기본값 회귀를 확인한다.
실제 두 backend worker에서 저장 RGB를 요청해 실제 결과와 metadata·정리를 확인한다.
Gazebo는 ROS_DOMAIN_ID=12/GZ_PARTITION=codex, GUI 없이 실행한다.
빈 문 지도와 재관측 비활성 상태의 완전 열림부터 실제 LiDAR SLAM·RGB-D 상태·지도 저장을 확인한다.
음성 벽 장면과 기존 기준선도 별도로 검사한다. 개발 장면의 실행을 독립·실물 성능으로 표현하지 않는다.
