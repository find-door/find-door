# Find Door — 문 위치·상태 시맨틱 지도

> **실내 지도 위에 문의 위치와 열림·닫힘 상태를 표시한다.**

기존 **Prometheus Mapping**의 2D LiDAR SLAM·RGB-D 투영 구조를 바탕으로,
문의 고정 위치와 열림·닫힘·판단 불가 상태를 지도에 표시하는 컴퓨터비전 과목 프로젝트입니다.
첫 단계로 Gazebo에서 문짝 각도·카메라 거리·관측각별 36조건·108프레임의 검출 실험을 완료했습니다.
문짝 검출과 고정 문틀 위치의 차이, 낮은 카메라 시점의 잘림 한계를 확인했습니다.
깊이 기반 단일 관측 후보와 [정답 ROI 개발 진단](docs/door-state-evaluation.md)을 구현했습니다.
[실시간 YOLOE·출입구 추정·시간 종합·LiDAR SLAM 연결](docs/door-live.md)을 단일 문 정지 관측에서 확인했습니다.
[순간 문 각도 전이](docs/door-transition.md), [짧은 직선 주행](docs/door-motion.md),
[작은 제자리 회전](docs/door-rotation.md)에서도 ID·상태를 확인했습니다.
문 검출 추가 학습 필요성 평가, 큰 회전·장거리·독립 장소·실제 촬영 자료 검증은 남아 있습니다.
[평판 금속문 개발 변형](docs/door-scene-variation.md)은 반열림만 성공했고 닫힘·완전 열림은 검출 실패로 남았습니다.
실행 계약·조건별 결과·실제 카메라 비교 그림은 [문 실험](docs/door-experiment.md)을 따릅니다.
Gazebo와 실제 촬영 영상의 평가는 별도로 기록합니다.

아래 `v0.1.0` 및 SLAM·시맨틱 검증 기록은 **분기 전 기반 프로젝트의 이력**입니다.
문 위치·상태 기능의 완료를 의미하지 않습니다. 드론 확장은 기존 장기 구상이며 이번 과목 범위에 포함하지 않습니다.

> **상태: 첫 릴리스 `v0.1.0` (2026-10-27) — 시뮬레이션 기준선·실물 배포 준비 완료, 로봇 도착 대기.** 자동 주행·지도 평가·루프 비교와 실물 전용 프로파일·점검·실행 경로를 구현했습니다. 루프 ON에서 관측한 지도 실패는 원인(스캔 무게중심 기반 루프 후보가 벽 너머 방을 연결)을 확정하고 기본 설정을 고쳤습니다([원인 분석](docs/loop-closure-investigation.md#2026-10-26-원인-분석-스캔-무게중심-기반-루프-후보-선택)). 이 수정은 `indoor_v1` 기준이며, 반복 구조의 `indoor_v2`에서는 현재 `slam.yaml`도 5회 모두 실패해 [조사 중](docs/loop-closure-investigation.md#2026-11-01-indoor_v2-실패-후보와-설정-검증)입니다. `slam.yaml`은 비교 기준이며 권장 설정이 아니고, 실물 기본값은 미선정입니다. 실물 MVP는 미완료입니다. [준비 범위](docs/prehardware-plan.md)와 검증 결과를 구분합니다.

![로봇이 시뮬레이션 실내를 주행하며 지도를 그리는 데모](docs/media/mapping_demo.gif)

*왼쪽: 로봇 카메라 / 오른쪽: RViz2의 slam_toolbox 지도와 SLAM 추정 경로(파란 선). Gazebo 시뮬레이션 `indoor_demo`, 3배속. 실물 주행이 아니며 평가 수치는 [evaluation.md](docs/evaluation.md)의 `indoor_v1` 결과를 기준으로 합니다.*

## 개요

| 항목 | 내용 |
|---|---|
| 기간 | 2026.10 ~ 진행 중 |
| 구분 | 컴퓨터비전 과목 프로젝트 (기존 매핑 프로젝트에서 분기) |
| 개발 플랫폼 | JetAuto Ultimate Kit (Jetson Orin NX 16GB, 4륜 메카넘) |
| 개발 환경 | Ubuntu 22.04, ROS 2 Humble, Gazebo Harmonic ([ADR-0002](docs/adr/0002-dev-environment.md)) |
| 기술 | Python, SLAM, RViz2 |
| 응용 목표 | 실내 지도에 문 위치·열림/닫힘 상태 표시 |

## 기반 프로젝트의 장기 과제

- **GPS 없음** — 실내에서는 센서만으로 위치를 추정해야 함 (SLAM)
- **드론의 기울기** — 2D LiDAR는 한 평면만 보므로, 기체가 기울면 바닥·천장을 벽으로 착각함
- **현장 환경** — 연기·어둠에서는 카메라가 거의 무용지물, 짙은 연기에서는 LiDAR도 노이즈 증가
- **통신 단절** — 건물 내부는 전파가 약해 지도를 실시간으로 밖에 전송하기 어려움
- **연산 제약** — 소형 기체에 실을 수 있는 Jetson급 컴퓨터에서 실시간 동작해야 함

## Find Door 목표 구조 (Gazebo 개발 검증, 독립·실물 평가는 예정)

```mermaid
flowchart LR
  L["2D LiDAR"] --> S["SLAM<br>2D 지도 + 위치"]
  O["휠 오도메트리 / IMU"] --> S
  C["RGB-D 카메라"] --> D["문 검출<br>깊이 기반 상태 판단"]
  S --> M["시맨틱 지도"]
  D --> M
  M --> R["RViz2 / 지휘소 화면"]
```

자세한 내용은 [docs/architecture.md](docs/architecture.md)를 참고하세요.

## 기반 프로젝트 로드맵 이력

현재 과목의 완료 조건은 [Find Door 로드맵](docs/roadmap.md#find-door-과목-프로젝트)을 따릅니다.
아래 표와 MVP 정의는 기존 프로젝트의 실물·드론 확장 계획입니다.

| 단계 | 완료 조건 | 상태 |
|---|---|---|
| Phase 0 개발 환경 | 실내 world에서 JetAuto 시뮬레이션 주행 | ✅ `v0.1.0` (RViz2 수동 확인·공개 범위 확인 남음) |
| Phase 1 2D SLAM | 실물로 만든 지도의 도면 대비 오차 측정 | 🚧 시뮬 기준선 완료, 실물 대기 |
| Phase 2 카메라 융합 | 지도 위 객체 표시 + 위치 정확도 측정, 데모 영상 | 🚧 시뮬 투영 검증 완료, 실물 대기 |
| Phase 3 3D (확장) | 3D 또는 층 구분 지도 생성 | ⏳ |
| Phase 4 드론 (확장) | 기울기 보정 매핑 검증 | ⏳ |

**MVP = Phase 1 + Phase 2.** 상세는 [docs/roadmap.md](docs/roadmap.md).

## 저장소 구조

```text
prometheus-mapping/
├── README.md
├── CONTRIBUTING.md        # 브랜치·커밋 규칙
├── .gitmessage            # 커밋 메시지 템플릿
├── .github/               # PR 템플릿·오프라인 CI
├── docs/
│   ├── architecture.md    # 시스템 구조, TF·토픽 설계
│   ├── interfaces.md      # 초기 센서·TF·실행 계약
│   ├── roadmap.md         # 단계별 목표·체크리스트
│   ├── hardware.md        # 로봇·센서 사양
│   ├── setup.md           # 개발 환경·실행 방법
│   ├── evaluation.md      # 평가 지표·실험 기록
│   └── adr/               # 설계 결정 기록 (ADR)
└── src/                   # ROS 2 패키지
    ├── mapping_gazebo/    # 매핑용 실내 world·launch ✅
    ├── mapping_slam/      # SLAM 기준선 설정·단독/통합 launch
    ├── mapping_semantic/  # 색상 표적·NanoDet 2D 검출, RGB-D 투영·객체 지도
    ├── mapping_eval/      # 입력 검사·자동 시뮬 주행·지도 기하 평가
    ├── mapping_robot/     # 실물 프로파일·읽기 전용 센서 점검·SLAM/선택 검출
    └── mapping_slam_debug/ # 시뮬 분석용 루프 수락 기록 솔버 플러그인
```

## 빠른 시작

```bash
source /opt/ros/humble/setup.bash
source ~/worker_following_ws/install/setup.bash   # wf_gazebo (연구과제, 오버레이)
colcon build --symlink-install && source install/setup.bash
ros2 launch mapping_slam simulation.launch.py
```

실물 설치·도착 당일 시험은 [로봇 배포 안내](docs/robot-deployment.md)를 따릅니다. 로봇에는 Gazebo 없는 두 런타임 패키지만 빌드합니다.

slam_toolbox 설치가 필요합니다. 초기 인터페이스는 [docs/interfaces.md](docs/interfaces.md)를 따릅니다.

자세한 내용은 [docs/setup.md](docs/setup.md), world 도면은 [mapping_gazebo README](src/mapping_gazebo/README.md).

## 검증 결과

기준선 3회 벽 P95 오차는 6.28 / 3.46 / 3.14 cm였습니다. 동일 bag에서 slam_toolbox·Cartographer 각 3회 재생과 합성 표적의 실제 RGB-D 투영·이동·저장·복원 검증을 수행했습니다. 이후 기록한 실패 bag으로 지도 급변 원인을 확정해 `use_scan_barycenter: false`로 고쳤고(온라인 5회 통과), 최초 예비 실행의 급변이 같은 원인인지는 미확정입니다. 실물 성능 결과가 아닙니다. [평가 기록](docs/evaluation.md)을 참고하세요.

## 연구과제와의 경계

| 구분 | 내용 |
|---|---|
| 연구과제에서 가져온 것 | JetAuto 로봇(하드웨어), Gazebo 시뮬레이션 모델·센서 설정(`wf_gazebo`) |
| 이 프로젝트에서 새로 만드는 것 | 매핑용 실내 world, SLAM 구성·평가, 카메라 융합(시맨틱 지도), 3D 확장 |
| 공개 범위 | 🔍 미정 — 교수님 확인 후 기록 (GitHub 공개 가능한 코드·자료 범위) |

## 기여 규칙

브랜치 전략(`main` / `dev` + 작업 브랜치)과 커밋 메시지 규칙은 [CONTRIBUTING.md](CONTRIBUTING.md)를 따릅니다.

## 라이선스

🔍 미정 — 공개 범위 확인 후 결정.
