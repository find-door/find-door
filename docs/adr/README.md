# 설계 결정 기록 (ADR)

"무엇을 왜 골랐나"를 남기는 곳입니다. 면접·발표에서 가장 많이 받는 질문에 대한 답이 됩니다.

- 파일 이름: `NNNN-짧은-제목.md` (예: `0003-slam-algorithm.md`)
- 새 결정은 [template.md](template.md)를 복사해서 작성
- 결정을 뒤집을 때는 기존 ADR을 지우지 않고, 새 ADR을 쓰고 기존 ADR 상태를 `대체됨 (ADR-NNNN)`으로 바꿉니다
- 브랜치: `docs/adr-NNNN-제목`, 커밋: `docs(adr): ...` ([CONTRIBUTING.md](../../CONTRIBUTING.md))

## 목록

| 번호 | 결정 항목 | 선택지 | 결정 | 상태 |
|---|---|---|---|---|
| [0001](0001-dev-platform-jetauto.md) | 개발 플랫폼 | 핸드헬드 리그 / 지상 로봇 / 드론 | JetAuto (지상 로봇) | ✅ 결정 |
| [0002](0002-dev-environment.md) | 개발 환경 | Humble + Fortress / Humble + Harmonic / Jazzy + Harmonic | Ubuntu 22.04 + ROS 2 Humble + Gazebo Harmonic | ✅ 결정 |
| 0003 | SLAM 알고리즘 | slam_toolbox / Cartographer / Hector SLAM (ROS 2 포트 확인 필요) | | 🔍 검토 중 |
| 0004 | 카메라 융합 방식 | 객체 인식 후 지도 투영 / RTAB-Map 등 통합 SLAM | | 🔍 검토 중 |
| 0005 | LiDAR 장착 방향 (3D 단계) | 수평 / 세로 / 비스듬히(30~45°) / 수평+세로 2개 | | 🔍 검토 중 |
| 0006 | 객체 인식 모델 | 🔍 (Jetson 실시간 동작 여부 기준) | | 🔍 검토 중 |
| 0007 | 객체 지도 갱신 정책 | 루프 클로저 시 재투영 / 완성 지도 위 위치 추정 모드로 제한 | | 🔍 검토 중 |

> 번호는 예정값입니다. 실제로 작성하는 순서대로 번호를 붙이고, 이 표와 [architecture.md](../architecture.md#7-미결정-사항)를 함께 갱신합니다.
