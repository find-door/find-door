# 로봇 도착 후 설치·첫 시험 실행

로봇용 런타임은 `mapping_robot`과 `mapping_semantic` 두 패키지입니다.
Gazebo·`wf_gazebo`·개발 PC의 `install/`은 로봇에 복사하지 않습니다.
현재 검증은 개발 PC의 x86_64 빌드·합성 ROS 입력입니다. **ARM64/Jetson·실제 드라이버·보정·물리 정지는 도착 후 확인해야 합니다.**

## 1. 개발 PC에서 전송할 소스 준비

최종 PR이 병합된 clean checkout에서 실행합니다. 출력은 저장소 밖의 새 파일입니다.

```bash
python3 scripts/robot_workspace.py bundle --output /tmp/prometheus-robot-new.tar.gz
sha256sum /tmp/prometheus-robot-new.tar.gz
```

bundle에는 두 런타임 패키지, 설치·기록 도구, 문서, 파일별 해시와 소스 커밋이 포함됩니다.
모델 가중치·드라이버·큰 bag·빌드 바이너리는 포함하지 않습니다. 제외한 개발 패키지의 상대 문서 링크는 해당 소스 커밋의 GitHub URL로 바꾸고, 코드 원본 해시와 bundle 파일 해시를 각각 보존합니다. 검증되지 않은 로컬 변경이 있으면
생성을 거부합니다. 모델을 사용할 때는 [검출 기준선](detector-baseline.md)의 FP32 ONNX와
기대 SHA-256을 함께 전송하고, 모델 출처·LICENSE를 보존합니다.

Git 접근이 되는 로봇에서는 같은 최종 커밋을 checkout해도 됩니다. 전송 주소·SSH 사용자명은
실제 장치에서 확인합니다. 로봇은 현재 연결돼 있지 않아 원격 설치를 실행하지 않았습니다.

## 2. 로봇의 환경 확인과 빌드

먼저 [인수 절차](robot-acceptance.md)의 장치·OS·전원·정지 수단을 확인합니다.
기준 환경은 Ubuntu 22.04 + ROS 2 Humble + 시스템 Python 3.10입니다.
JetPack/L4T·카메라 드라이버가 다른 OS를 요구하면 기존 시스템을 유지하고 호환 계획을 먼저
기록합니다. 이 절차는 OS/JetPack을 재설치하거나 전원 모드를 바꾸지 않습니다.
ROS 초기 설치가 필요하면 [ROS 2 공식 Humble 설치 안내](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)를 따릅니다.

```bash
# 로봇에서 source bundle을 풀고 그 루트로 이동한 상태
python3 scripts/robot_workspace.py verify-bundle .
source /opt/ros/humble/setup.bash

# ROS apt 저장소가 준비된 Ubuntu 22.04에서 빌드 도구 설치
sudo apt-get update
sudo apt-get install python3-colcon-common-extensions python3-rosdep python3-yaml
# 이미 초기화된 rosdep은 유지
if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
rosdep install --from-paths src/mapping_robot src/mapping_semantic \
  --ignore-src --rosdistro humble -y
colcon build --packages-select mapping_semantic mapping_robot
source install/setup.bash

mkdir -p ~/prometheus-results/arrival
python3 scripts/robot_workspace.py check-env \
  --output ~/prometheus-results/arrival/environment.json
```

전체 저장소를 받아도 위 두 패키지만 빌드합니다. 이 경로에는 `mapping_gazebo`, `mapping_slam`,
`mapping_eval`이 필요하지 않습니다. 지도 저장에는 `nav2_map_server`, bag에는 ROS 2 CLI·rosbag2와
SQLite3 저장 플러그인이 필요하며 이들은 런타임 의존성으로 관리합니다.
`check-env`는 OS/Python/ROS·필수 패키지를 확인하고 Jetson 모델·L4T·디스크 등 호스트 정보를
기록합니다. 환경 통과가 드라이버·센서·ARM64 전체 인수 통과를 뜻하지 않습니다.

## 3. 실제 센서 설정 파일 채우기

```bash
mkdir -p ~/prometheus-config
cp src/mapping_robot/config/robot.template.yaml ~/prometheus-config/robot.yaml
ros2 topic list -t
# 확인한 실제 센서마다 타입·QoS·header·주기를 조사
ros2 topic info --verbose /실제_scan_토픽
ros2 topic echo --once /실제_scan_토픽
```

외부 드라이버에서 확인한 scan/odom 토픽과 map·odom·base·laser 프레임,
센서의 측정 범위, 지도 해상도, 최소 수신률·최대 지연을 YAML에 입력합니다.
`hardware_confirmed`는 실제 자료를 확인한 뒤에만 true로 바꿉니다.
미정 값은 검증기가 거부하며 시뮬레이션 장착 치수나 센서값으로 자동 채우지 않습니다.
SLAM-only에서는 optical frame과 카메라 토픽을 `null`, `semantic.enabled=false`로 둬도 됩니다.

첫 실물 시험 프로파일은 `do_loop_closing=false`를 요구합니다.
새 기록에서 ON 지도 실패·OFF 정상의 차이를 확인했기 때문에 선택한 보수적 초기 조건입니다.
장거리 누적 오차를 해결하거나 안정성을 보장하는 설정은 아닙니다.
[실패 분석](slam-startup-capture.md)과 실물 반복 시험을 바탕으로 이후 설정을 따로 결정합니다.

```bash
ros2 run mapping_robot validate_profile --profile ~/prometheus-config/robot.yaml
ros2 run mapping_robot preflight --profile ~/prometheus-config/robot.yaml \
  --duration 10 --output ~/prometheus-results/arrival/sensors.json
```

이 검사는 구동 명령을 발행하지 않습니다. 충분한 관측기간·수신률·header 진행/최근성·타입·QoS·발행자,
scan 시각의 TF와 odom pose/동일 시각 TF 일치를 검사합니다. 실물 모드에 `/clock`이 있으면 실패합니다.
구체적인 검사 범위·수치 허용차·TF authority 검사 한계는 [패키지 설명](../src/mapping_robot/README.md)에 있습니다.
실패한 경우 보고서의 항목을 해결하고 **새 결과 파일명**으로 다시 검사합니다.

## 4. 기록을 먼저 시작하고 SLAM 연결

아래 첫 기록·SLAM 순서는 `semantic.enabled=false`인 초기 SLAM 시험입니다. 먼저 실제 드라이버가 정지 상태로 동작하고 물리 정지 수단을 확인한 상태에서 진행합니다.
각 터미널은 로봇의 같은 ROS 도메인과 현재 `install/setup.bash`를 사용합니다.
사용자는 실측 네트워크 환경에 맞는 ROS_DOMAIN_ID를 정하며, 에이전트의 검증 도메인 12를
로봇 운용값으로 강제하지 않습니다.

```bash
# 터미널 A: 실제 센서·TF만 기록(명령 토픽 제외), Ctrl-C로 종료 가능
python3 scripts/robot_recording.py record \
  --profile ~/prometheus-config/robot.yaml \
  --output ~/prometheus-results/arrival/stationary-bag --duration 30

# 터미널 B: SLAM만 연결. 드라이버나 이동 제어기는 시작하지 않음
ros2 launch mapping_robot robot.launch.py profile:=$HOME/prometheus-config/robot.yaml

# 터미널 C: 지도 저장(새 출력 이름)
ros2 run nav2_map_server map_saver_cli -f ~/prometheus-results/arrival/map \
  --ros-args -p use_sim_time:=false
```

정지 입력과 지도부터 확인하고, 실제 이동은 [실물 인수](robot-acceptance.md)의
정지 확인 → 현장 감독 저속 직진·회전·횡이동 순서로 진행합니다.
시뮬레이션의 `drive_route`를 실물 제어기로 사용하지 않습니다.
원격 속도 0 명령은 물리 정지의 대체 수단이 아닙니다.
IMU 기록·구동 watchdog·휠/IMU 융합은 실제 드라이버 계약을 확인한 뒤 추가합니다.

## 5. 카메라와 객체 지도를 켤 때

먼저 RGB와 depth가 **물리적으로 정렬된 토픽**인지, 올바른 RGB 기준 CameraInfo인지,
장착 TF·왜곡 보정이 맞는지 실측합니다. 단순히 같은 해상도·frame인 것으로 정렬을 확정하지 않습니다.
지원 입력은 rgb8/bgr8, `32FC1` 미터 또는 `16UC1` 밀리미터입니다.
내부에서 미터로 정규화하며 0·NaN·Inf와 범위 밖 깊이는 투영에 쓰지 않습니다.
왜곡된 raw RGB나 RGB에 정렬되지 않은 depth는 드라이버의 정렬/보정 출력이 필요합니다.

프로파일에 실제 카메라 토픽·optical frame과 `calibration_id`, `registration_note`,
`aligned_depth_confirmed=true`, 모델 파일·기대 SHA-256, 새 객체 JSON 경로와 `map_id`를 입력합니다.
`semantic.enabled=true`일 때만 검출기와 external mapper를 함께 실행합니다.
semantic 활성 프로파일로 기록할 때는 검출기까지 준비한 뒤 기록을 시작합니다. 검출 토픽도 필수 입력으로 검사합니다. SLAM 시작 전 raw RGB-D까지 먼저 확보하려면 카메라 토픽을 채운 별도 semantic 비활성 기록 프로파일을 사용하고 두 프로파일·해시를 함께 보존합니다.
초기 정적 누적 대상은 chair/cup/bottle의 부분집합이고 person은 관측만 합니다.
현재 모델에는 문·계단·소화기 클래스가 없습니다.

기본 `sync_policy=exact`입니다. 실제 센서의 시간차를 측정한 뒤 필요한 경우에만
`approximate`와 `max_sync_skew_sec`를 명시합니다(최대 0.1초).
RGB와 검출은 항상 exact header이고 depth·CameraInfo의 편차만 제한적으로 허용합니다.
TF 조회·객체 출력 시각은 RGB 시각입니다. stamp=0 CameraInfo를 임의로 재사용하지 않습니다.

```bash
ros2 run mapping_robot preflight --profile ~/prometheus-config/robot.yaml \
  --require-camera --duration 10 --output ~/prometheus-results/arrival/rgbd.json
ros2 launch mapping_robot robot.launch.py profile:=$HOME/prometheus-config/robot.yaml
# 객체 저장은 서비스 요청 시 수행
ros2 service call /semantic/save std_srvs/srv/Trigger '{}'
```

## 6. 기록을 개발 PC에서 재생하고 자원 기록

실물 bag에는 `/clock`이 없으므로 기존 시뮬레이션 `replay_benchmark`에 직접 넣지 않습니다.
새 기록 도구는 프로파일에 명시한 센서·odom→base·장착 정적 TF만 별도 bag으로 보존하고,
과거 map→odom·구동 명령을 제거합니다. 재생 player가 단일 `/clock`을 생성합니다.

```bash
# 로봇 구동 연결이 없는 개발 PC의 격리 도메인에서 센서만 재생
export ROS_DOMAIN_ID=12 GZ_PARTITION=codex
python3 scripts/robot_recording.py replay --profile /absolute/path/robot.yaml \
  --bag /absolute/path/hardware-bag --output /tmp/hardware-replay-new \
  --confirm-isolated
```

재생 SLAM 수신 노드는 별도 터미널에서 player보다 먼저 준비합니다. 기본 허용 수신 노드는 `/slam_toolbox`이며 다른 이름을 쓸 때는 `--receiver-node`를 명시합니다. 기존 실물 센서·입력 TF가 실행 중이면 재생을 거부합니다.

이는 센서 player이며 실물 launch는 `use_sim_time=false`로 고정되어 재생에 쓰지 않습니다.
재생 SLAM은 개발 PC의 `mapping_slam mapping.launch.py use_sim_time:=true`에 실제 frame/scan
설정을 적용한 별도 파라미터 파일을 연결합니다. semantic도 재생에서는 `use_sim_time=true`를 사용합니다.
실물 bag의 독립 정답 기반 지도 품질 평가는 실제 공간·보정·정답 수집 후 수행합니다.

```bash
# SLAM·카메라를 실행하는 로봇에서 OS 자원 원자료 수집
python3 scripts/robot_workspace.py sample-resources \
  --duration 60 --interval 1 --output ~/prometheus-results/arrival/resources
```

자원 도구는 CPU/memory 원자료·온도·시각·호스트 정보를 기록하고 합격을 자동 선언하지 않습니다.
Jetson GPU·전력은 실제 `tegrastats` 등으로 별도 측정하고 추론 지연은 detector/semantic status와
함께 남깁니다. 전원 모드·냉각·시험 시간도 인수 결과에 적습니다.

개발 PC에서 실제 수행한 시험·실패와 보완·원자료는 [배포 준비 검증](robot-readiness-evaluation.md)에 있습니다.
