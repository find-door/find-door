# 설계 결정 기록 (ADR)

"무엇을 왜 골랐나"를 남기는 곳입니다. 면접·발표에서 가장 많이 받는 질문에 대한 답이 됩니다.

- 파일 이름: `NNNN-짧은-제목.md` (예: `0004-slam-algorithm.md`)
- 새 결정은 [template.md](template.md)를 복사해서 작성
- 결정을 뒤집을 때는 기존 ADR을 지우지 않고, 새 ADR을 쓰고 기존 ADR 상태를 `대체됨 (ADR-NNNN)`으로 바꿉니다
- 브랜치: `docs/adr-NNNN-제목`, 커밋: `docs(adr): ...` ([CONTRIBUTING.md](../../CONTRIBUTING.md))

## 목록

| 번호 | 결정 항목 | 선택지 | 결정 | 상태 |
|---|---|---|---|---|
| [0001](0001-dev-platform-jetauto.md) | 개발 플랫폼 | 핸드헬드 리그 / 지상 로봇 / 드론 | JetAuto (지상 로봇) | ✅ 결정 |
| [0002](0002-dev-environment.md) | 개발 환경 | Humble + Fortress / Humble + Harmonic / Jazzy + Harmonic | Ubuntu 22.04 + ROS 2 Humble + Gazebo Harmonic | ✅ 결정 |
| [0003](0003-wf-gazebo-reuse.md) | `wf_gazebo` 재사용 방식 | 오버레이 의존 / 저장소에 복사 / 별도 저장소 | 오버레이 의존 (`worker_following_ws`) | ✅ 결정 |
| [0004](0004-prehardware-slam-baseline.md) | SLAM 개발 기준선 | slam_toolbox / Cartographer / Hector | slam_toolbox 기본·Cartographer 선택 비교, 루프 후보 로봇 위치 기준, 실물 유보 | ✅ 시뮬 기준선 결정 |
| [0005](0005-rgbd-projection.md) | 카메라 융합 | 별도 검출·투영 / 통합 RGB-D SLAM | Detection2D + 정렬 깊이 + 관측 시각 TF | ✅ 결정 |
| 0006 | LiDAR 장착 방향 (3D 단계) | 수평 / 세로 / 비스듬히(30~45°) / 수평+세로 2개 | | 🔍 검토 중 |
| [0007](0007-cpu-detector-baseline.md) | 검출 개발 기준선 | 색상 표적 / NanoDet / 실물 목표 모델 | NanoDet CPU 연결 기준선, 실물 목표 모델 유보 | ✅ CPU 기준선 결정 |
| [0008](0008-semantic-map-policy.md) | 객체 지도 정책 | 재투영 / 고정 지도 / epoch 초기화 | 정적 객체·고정 지도 우선·큰 보정 초기화 | ✅ 제한 범위 결정 |
| [0009](0009-door-ros-pipeline.md) | 문 상태 ROS 연결 | 일반 객체 / 문 전용 노드 / 전용 메시지 | 문 위치·상태 분리, 관측 시각 TF, 개발 JSON·마커 | ✅ 개발 연결 결정 |
| [0010](0010-live-door-detector.md) | 실시간 검출 연결 | 프레임별 CLI / ROS 직접 적재 / 상주 worker | 별도 가상환경 worker와 원본 RGB-D 묶음 발행 | ✅ 로컬 개발 연결 |
| [0011](0011-door-dataset-contract.md) | 라벨·분할 계약 | 단일 bbox / 이중 bbox / 마스크 | 문짝·출입구 분리, RGB 해시·장면 그룹 누출 검증 | ✅ 개발 자료 계약 |
| [0012](0012-known-door-reobservation.md) | 알려진 출입구 재관측 | 매 프레임 검출 / 과거 상태 재사용 / 기하 재투영 | 검출로 확인한 기하와 현재 RGB-D 대조, 기존 ID만 갱신 | ✅ 제한적 개발 구현 |
| [0013](0013-dual-box-evaluation.md) | 검수 bbox 진단 | 문틀 단일 / 두 기준별 / 합산 | 문짝·출입구 기준별 대응, 검수·환경·오류 분모 분리 | ✅ 평가 도구 계약 |
| [0014](0014-yolo11-live-candidate.md) | 학습 후보 실시간 연결 | 기본 교체 / ROS 적재 / worker 설정 분기 | 로컬 YOLO11 출입구 후보의 선택 실행, YOLOE 기본 유지 | ✅ 개발 연결 결정 |

| [0015](0015-door-last-confirmed-state.md) | 마지막 문 확인 이력 | TTL 연장 / state 대체 / 이력 분리 | 현재 판단과 마지막 확정 상태·시각 분리, schema2·과거 표시 | ✅ 사용자 요청 수정 |

> 번호는 예정값입니다. 실제로 작성하는 순서대로 번호를 붙이고, 이 표와 [architecture.md](../architecture.md#7-미결정-사항)를 함께 갱신합니다.
