# 개발 환경·실행 방법

다른 사람이 따라 해도 재현되도록 씁니다. 명령을 바꾸면 실제로 한 번 실행해 보고 아래 「확인 상태」 날짜를 갱신합니다.

## 환경

| 구분 | OS / ROS | 비고 |
|---|---|---|
| 개발 PC (시뮬레이션) | Ubuntu 22.04 / ROS 2 Humble | Gazebo Harmonic + `ros_gz` (`ros-humble-ros-gzharmonic`) |
| 로봇 (JetAuto) | Ubuntu 22.04 / ROS 2 Humble | Jetson Orin NX 16GB |

결정 근거: [ADR-0002](adr/0002-dev-environment.md)

> ⚠️ Humble의 공식 기본 조합은 Gazebo Fortress이며, Harmonic은 **비기본 조합**입니다. `ros-humble-ros-gzharmonic`은 Fortress용 `ros-humble-ros-gz*` 및 Gazebo에 의존하는 공식 ROS 패키지와 **충돌**하므로 함께 설치하지 않습니다.

## 외부 의존성·버전 기록

재현성을 위해 아래를 확정 후 기록합니다.

| 항목 | 출처·접근 경로 | 버전 / revision |
|---|---|---|
| `wf_gazebo` (연구과제 모델·센서 설정) | 로컬 `~/worker_following_ws/src/wf_gazebo` (오버레이, [ADR-0003](adr/0003-wf-gazebo-reuse.md)) | git 미관리 → 버전 대신 확인 날짜 기록: 2026-10-21 (`rosorin.gz.xacro` 수정 시각 2026-10-15) |
| 로봇 URDF·메시 (Hiwonder) | 🔍 | 🔍 |
| JetPack / L4T | — | 🔍 |
| 실물 로봇·센서 드라이버 | 🔍 | 🔍 |

> `wf_gazebo`를 공개할 수 없으면 대체 모델(공개 URDF + 동일 센서 설정)을 둘지 결정 🔍

## 설치

### 1. ROS 2 Humble

[ROS 2 Humble 설치 문서](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)를 따라 `ros-humble-desktop`을 설치합니다.

### 2. Gazebo Harmonic

출처: [Gazebo Harmonic 바이너리 설치](https://gazebosim.org/docs/harmonic/install_ubuntu/)

```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

### 3. ROS 2 ↔ Gazebo 연동 (`ros_gz`)

출처: [Gazebo — ROS 2 설치 (Humble + Harmonic)](https://gazebosim.org/docs/harmonic/ros_installation/)

```bash
# Fortress용 ros-humble-ros-gz* 가 설치되어 있으면 먼저 제거
sudo apt-get install ros-humble-ros-gzharmonic
```

### 설치 확인

```bash
source /opt/ros/humble/setup.bash
ros2 pkg list | grep ros_gz          # ros_gz, ros_gz_bridge, ros_gz_sim ... 출력
gz sim --versions                    # 8.x (= Harmonic)

# Gazebo → ROS /clock 브리지 확인 (터미널 3개)
gz sim -s -r empty.sdf
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
ros2 topic echo --once /clock        # sec/nanosec가 출력되면 정상
```

개발 PC 확인 결과 (2026-10-21):

| 항목 | 버전 |
|---|---|
| OS | Ubuntu 22.04.5 LTS |
| ROS 2 | `ros-humble-desktop` 0.10.0 |
| Gazebo | Gazebo Sim 8.15.0 (`libgz-sim8`, Harmonic) |
| ros_gz | `ros-humble-ros-gzharmonic` 0.244.12 (Fortress용 `ros-humble-ros-gz*` 미설치 ✅) |
| 빌드 도구 | `colcon`, `rosdep` 설치됨 |

> `gz-harmonic` 메타패키지 없이 `libgz-sim8`만으로도 위 확인은 통과했습니다. GUI 등 추가 도구가 필요하면 `sudo apt-get install gz-harmonic`.

### 4. 프로젝트 의존성

```bash
# 초기 SLAM 기준선. Fortress용 ros_gz를 추가 설치하지 않음
sudo apt-get install ros-humble-slam-toolbox
```

## 빌드

`wf_gazebo`(연구과제)를 오버레이로 사용하므로 `worker_following_ws`가 먼저 빌드되어 있어야 합니다 ([ADR-0003](adr/0003-wf-gazebo-reuse.md)).

```bash
source /opt/ros/humble/setup.bash
source ~/worker_following_ws/install/setup.bash
# 현재 checkout 또는 worktree 루트에서 실행
colcon build --symlink-install
```

## 실행 — 시뮬레이션

새 터미널마다 아래 3줄을 먼저 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/worker_following_ws/install/setup.bash
source install/setup.bash  # 현재 checkout/worktree 루트
```

```bash
# 터미널 1 — Gazebo + 로봇 (indoor_v1 world, 복도 시작점 스폰)
ros2 launch mapping_gazebo indoor.launch.py          # GUI 없이: gui:=false

# 터미널 2 — SLAM
# 🔍

# 터미널 3 — RViz2
# 🔍

# 터미널 4 — 주행 (teleop)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.2
```

world 도면과 기대 센서값은 [mapping_gazebo README](../src/mapping_gazebo/README.md)에 있습니다.

## 실행 — 실물 JetAuto

```bash
# 🔍 로봇 드라이버 실행, SLAM 실행, 지도 저장
```

## 지도 저장·불러오기

```bash
# 🔍
```

## 자주 생기는 문제

「증상 → 원인 → 해결 명령」 형식으로 추가합니다.

- **launch를 Ctrl+C가 아닌 방법으로 끄면 `gz sim`이 남음** → 다음 실행 때 토픽이 섞임 → `pkill -INT -f '^gz sim'`
- **`libEGL warning: egl: failed to create dri2 screen`** (GUI 없이 실행할 때) → 센서 토픽이 정상으로 나오면 무시해도 됨. 센서가 안 나오면 `headless_rendering:=false`

> 확인 상태: 설치·설치 확인 — 2026-10-21 개발 PC에서 확인 / 빌드·시뮬레이션 실행(터미널 1, `gui:=false`) — 2026-10-21 확인 / SLAM·RViz2 — 🔍

## 초기 패키지 구조 실행

[인터페이스 계약](interfaces.md), [SLAM 실행·지도 저장](../src/mapping_slam/README.md),
[입력 검사](../src/mapping_eval/README.md)를 참고합니다.

```bash
# Codex 검증용 격리: 관련 터미널마다 동일하게 설정
export ROS_DOMAIN_ID=12 GZ_PARTITION=codex
ros2 launch mapping_slam simulation.launch.py
# 별도 터미널에서 환경 준비 후
ros2 run mapping_eval check_interfaces --timeout 30 --require-map
```

통합 launch를 사용하면 위 시뮬 단독 launch를 함께 실행하지 않습니다.

초기 패키지 빌드·통합 SLAM·지도 저장/재로딩은 2026-10-25 확인했습니다.
검증 범위와 제한은 [평가 기록](evaluation.md#초기-패키지-연결-검증--2026-10-25)에 남겼습니다.
