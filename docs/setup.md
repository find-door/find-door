# 개발 환경·실행 방법

다른 사람이 따라 해도 재현되도록 씁니다. 실행 결과는 [평가 기록](evaluation.md)에 남기고, 새 설치·실물 절차와 이미 실행한 검증을 구분합니다.

## 환경

| 구분 | OS / ROS | 비고 |
|---|---|---|
| 개발 PC (시뮬레이션) | Ubuntu 22.04 / ROS 2 Humble | Gazebo Harmonic + `ros_gz` (`ros-humble-ros-gzharmonic`) |
| 로봇 (JetAuto, 목표) | Ubuntu 22.04 / ROS 2 Humble | Jetson Orin NX 16GB; 실제 이미지·JetPack 확인 🔍 |

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
# 설치 계획에 Fortress 패키지 제거·예상 밖 의존성이 있는지 검토한 뒤 적용
sudo apt-get install ros-humble-ros-gzharmonic
```

### 설치 확인

```bash
source /opt/ros/humble/setup.bash
ros2 pkg list | rg ros_gz          # ros_gz, ros_gz_bridge, ros_gz_sim ... 출력
gz sim --versions                    # 8.x (= Harmonic)

# Gazebo → ROS /clock 브리지 확인 (터미널 3개)
gz sim -s -r empty.sdf
ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
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
sudo apt-get install python3-colcon-common-extensions python3-rosdep \
  python3-pytest python3-numpy python3-scipy python3-pil python3-yaml python3-opencv \
  ros-humble-slam-toolbox ros-humble-nav2-map-server \
  ros-humble-rosbag2-transport ros-humble-rosbag2-py ros-humble-rosbag2-interfaces \
  ros-humble-ros2bag ros-humble-rosbag2-storage-default-plugins \
  ros-humble-tf2-ros ros-humble-tf2-msgs ros-humble-rosgraph-msgs \
  ros-humble-cv-bridge ros-humble-message-filters ros-humble-vision-msgs \
  ros-humble-visualization-msgs ros-humble-std-srvs ros-humble-ros2param \
  ros-humble-teleop-twist-keyboard
```

`ros-humble-desktop`의 기본 메시지·launch·rclpy 패키지와 위 추가 의존성을 사용합니다. `wf_gazebo`는 공개 다운로드 대상으로 가정하지 않으며, 이미 준비된 외부 오버레이를 읽기 전용으로 사용합니다. `rosdep install`을 일괄 적용하기 전에는 비기본 Harmonic 조합에 Fortress 의존성이 들어오는지 검토합니다. 위 전체 설치 명령을 깨끗한 새 PC에서 재현한 결과는 아직 없습니다 (`🔍`).

## 빌드

`wf_gazebo`(연구과제)를 오버레이로 사용하므로 `worker_following_ws`가 먼저 빌드되어 있어야 합니다 ([ADR-0003](adr/0003-wf-gazebo-reuse.md)).

```bash
source /opt/ros/humble/setup.bash
source ~/worker_following_ws/install/setup.bash
# 현재 checkout 또는 worktree 루트에서 실행
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

## 실행 — 시뮬레이션

새 터미널마다 아래 3줄을 먼저 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/worker_following_ws/install/setup.bash
source install/setup.bash  # 현재 checkout/worktree 루트
```

에이전트는 해당 세션의 격리 값을 모든 터미널에 적용합니다. 사용자 GUI는 기본 도메인 0을 사용하며 직접 실행합니다. Claude Code는 `ROS_DOMAIN_ID=11 GZ_PARTITION=claude`, Codex는 아래 값을 씁니다.

```bash
export ROS_DOMAIN_ID=12 GZ_PARTITION=codex
# 백그라운드 launch와 자식 프로세스만 종료할 수 있도록 그룹 추적
set -m
ros2 launch mapping_slam simulation.launch.py gui:=false &
MAPPING_PGID=$!

# 환경·도메인을 동일하게 준비한 별도 터미널에서 검사
ros2 run mapping_eval check_interfaces --timeout 30 --require-map
# 검사 후, launch를 시작한 셸에서 종료
kill -INT -- -"$MAPPING_PGID"
wait "$MAPPING_PGID"
```

종료 신호 후에도 그룹이 남는지 `ps -g "$MAPPING_PGID" -o pid,pgid,args`로 확인합니다. 본인이 이번 실행에서 만든 그룹임을 확인한 경우에만 같은 그룹에 TERM을 보냅니다. 오래된 PID를 재사용하거나 이름으로 다른 ROS/Gazebo 프로세스를 종료하지 않습니다.

통합 launch는 Gazebo와 SLAM을 함께 시작하므로 시뮬레이터를 추가 실행하지 않습니다. 이미 실행 중인 시뮬레이터에는 `ros2 launch mapping_slam mapping.launch.py use_sim_time:=true`만 연결합니다. GUI가 필요한 사용자는 통합 launch의 `gui:=true`를 직접 선택합니다. RViz 화면 구성은 별도 수동 확인 대상입니다 (`🔍`).

수동 주행이 필요할 때 같은 환경의 터미널에서 실행합니다.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.2
```

world 도면·센서는 [mapping_gazebo](../src/mapping_gazebo/README.md), 지도 저장·재로딩 명령은 [mapping_slam](../src/mapping_slam/README.md#지도-저장과-재로딩), 자동 평가·기록은 [mapping_eval](../src/mapping_eval/README.md)을 참고합니다.

## 실행 — 실물 JetAuto

실물 드라이버·센서 계약은 아직 확인하지 않았습니다. [로봇 인수 절차](robot-acceptance.md)를 먼저 수행하고, 실제 토픽·TF·파라미터를 확정한 뒤 `use_sim_time=false`로 연결합니다. 시뮬레이션의 주행 스크립트와 장착 치수를 실물에 그대로 적용하지 않습니다.

## ROS 없는 코드·문서 검사

[offline.yml](../.github/workflows/offline.yml)은 Ubuntu 22.04의 시스템 Python 3.10과 apt 패키지로 다음 검사를 수행합니다. 외부 `wf_gazebo`, Gazebo, ROS 노드는 실행하지 않습니다. 설치 단계에는 네트워크가 필요하며, 여기서 오프라인은 센서·ROS 실행 없이 가능한 검사를 뜻합니다.

```bash
/usr/bin/python3 -m compileall -q src scripts
/usr/bin/python3 scripts/check_docs.py
PYTHONPATH=src/mapping_eval:src/mapping_semantic /usr/bin/python3 -m pytest src/mapping_eval/test src/mapping_semantic/test -q
```

문서 검사는 상대 inline 링크의 파일과 Markdown 제목 앵커를 확인합니다. 외부 URL·참조형 링크와 GitHub 렌더링 전체를 검증하지는 않습니다. 이 CI는 ROS 빌드·실제 토픽/QoS·TF·기록/재생·시뮬레이션 실행 및 실물 검증을 대체하지 않습니다. 워크플로 원격 실행 결과는 GitHub에서 별도로 확인합니다.

## 자주 생기는 문제

「증상 → 원인 → 해결 명령」 형식으로 추가합니다.

- **종료 후 Gazebo가 남거나 `/clock` 발행자가 둘 이상임** → 자기 launch의 프로세스 그룹과 도메인·partition 확인 → 위 종료 절차로 자기 그룹만 정리. 다른 세션의 프로세스에는 신호를 보내지 않음.
- **`libEGL warning: egl: failed to create dri2 screen`** (GUI 없이 실행할 때) → 센서 토픽이 정상으로 나오면 무시해도 됨. 센서가 안 나오면 `headless_rendering:=false`

> 확인 상태: 기본 설치·시뮬레이션 — 2026-10-21 개발 PC 확인. 초기 통합 SLAM·지도 저장/재로딩 — 2026-10-25 확인. 새 PC 전체 설치·RViz 구성·실물 실행 — 🔍. 최근 실행 근거는 [평가 기록](evaluation.md)을 기준으로 합니다.

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
