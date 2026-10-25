# 초기 인터페이스 계약

이 계약은 `mapping_gazebo` → `mapping_slam` → `mapping_eval`의 시뮬레이션 기준입니다.
실물 센서 계약은 아직 미정이며 시맨틱 관측·객체에는 표준 vision_msgs를 사용합니다. 표준 ROS 메시지로 충분한 현재 단계에서는
별도 `mapping_interfaces` 메시지 패키지를 만들지 않습니다.

## 패키지 경계

| 패키지 | 소유하는 것 | 소유하지 않는 것 |
|---|---|---|
| `mapping_gazebo` | 실내 world, 외부 launch wrapper | 로봇 모델·센서·브리지 구현 |
| `mapping_slam` | SLAM 설정, 단독·통합 launch | 센서 드라이버, odom·고정 TF 발행 |
| `mapping_eval` | 입력 검사, 시뮬 전용 경로 주행·반복 실행, 지도 기하 평가 | 실물 제어기, SLAM 알고리즘, 독립적인 ATE |
| `mapping_semantic` | NanoDet 2D 검출, 색상 표적·외부 검출 RGB-D 투영·정적 객체 지도 | 실물 보정, SLAM, 로봇 제어, 동적 추적 |

## 센서·출력 토픽

아래 센서 이름·타입·프레임은 외부 `wf_gazebo`의 `config/gz_bridge.yaml`,
`urdf/rosorin.gz.xacro`, `urdf/mecanum.xacro`와 launch를 읽어 확인했습니다.
명목 주기는 시뮬레이션 시간 기준이며 실시간 성능 보증이 아닙니다.

| 토픽 | 메시지 타입 | 프레임·역할 | 명목 주기 |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS, 발행자 하나 | 시뮬 시간 |
| `/scan` | `sensor_msgs/msg/LaserScan` | `lidar_sim_frame`, 360빔, 0.15–12 m | 5 Hz |
| `/odom` | `nav_msgs/msg/Odometry` | `odom` → `base_footprint` | 20 Hz |
| `/imu_data` | `sensor_msgs/msg/Imu` | `imu_link` | 100 Hz |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | `camera_color_optical_frame` | 15 Hz |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | `camera_color_optical_frame` | 15 Hz |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | RGB-D 내부 보정 | 카메라 기준 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | body 기준 x·y 속도와 yaw 각속도, ROS → Gazebo | 명령 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | `map`, SLAM 출력 | 설정상 갱신 간격 1 s |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | 아래 발행 주체 표 참조 | 동적 / 정적 |

브리지 발행 QoS는 외부 YAML에 명시되어 있지 않으므로 `ros2 topic info -v`로 확인합니다.
2026-10-25 런타임에서 `/scan`·`/clock` 발행은 RELIABLE/VOLATILE, 각 발행자 1개를 확인했습니다.
검사기는 센서 입력에 BEST_EFFORT/VOLATILE을 사용하고 `/map`에는 RELIABLE/TRANSIENT_LOCAL을 사용합니다.
Gazebo 런타임에서 rgb8·32FC1(m), 640×480, fx=fy=499.409, 공통 optical frame, 무왜곡 K와 exact-time 동기화를 확인했습니다. NaN·inf·0 및 유효 범위 밖 깊이를 거부합니다. 이 값은 실물 보정 결과가 아닙니다.

## TF와 시간

| 변환 | 유일한 발행 주체 |
|---|---|
| `map → odom` | 선택한 SLAM 하나: slam_toolbox 또는 Cartographer |
| `odom → base_footprint` | Gazebo OdometryPublisher → 외부 브리지 |
| `base_footprint → base_link → 센서` | 외부 launch의 robot_state_publisher |

외부 launch가 Gazebo·스폰·브리지·robot_state_publisher를 실행합니다.
이 프로젝트에서 같은 노드를 추가로 실행하지 않습니다. slam_toolbox의 odom 입력은 TF이며,
Cartographer 비교 설정은 `/odom` 메시지도 구독합니다.
센서 처리는 수신 시각이 아닌 메시지 타임스탬프로 TF를 조회합니다.

**현재 시뮬레이션은 VelocityControl로 body 속도를 직접 적용합니다.** `/odom`은 Gazebo 자세 기반이며
휠 엔코더 기반이 아닙니다. 이 환경으로 실제 메카넘 미끄러짐·오도메트리 오차를 검증했다고 주장하지 않습니다.
센서 프레임 이름은 기본 JetAuto 구성으로 고정하며, 다른 로봇 연결 시 계약과 설정을 함께 바꿉니다.

## 실행 계약

- `mapping_gazebo indoor.launch.py`: 기존 시뮬만 실행.
- `mapping_slam mapping.launch.py`: 기존 센서·TF에 SLAM만 연결. `use_sim_time` 기본값은 `false`.
- `mapping_slam simulation.launch.py`: 시뮬과 SLAM을 함께 실행. `use_sim_time=true`, GUI 기본값 `false`.
- 통합 launch와 시뮬 단독 launch를 같은 도메인에서 동시에 실행하지 않습니다.
- rosbag 재생 시에는 시뮬을 끄고 단독 SLAM을 `use_sim_time:=true`로 실행합니다.
  rosbag이 `/clock`을 포함하면 `--clock`으로 두 번째 시계를 만들지 않습니다.
  재실행 입력에는 이전 SLAM의 `map → odom`을 섞지 않습니다.

Codex 검증은 `ROS_DOMAIN_ID=12`, `GZ_PARTITION=codex`로 격리합니다.
사용자 GUI/RViz도 동일 실행을 볼 때만 같은 도메인을 사용합니다.

## 검사 범위와 후속 작업

`ros2 run mapping_eval check_interfaces --timeout 30 --require-map`으로
핵심 입력 수신·프레임·시간 진행·TF 연결·지도 메시지를 검사합니다.
이는 지도 정확도, 카메라 보정, 모든 TF 발행자 중복, 실제 센서 주파수 검증을 대신하지 않습니다.
정확도 평가는 [evaluation.md](evaluation.md), 시맨틱 메시지 설계는 ADR-0005·0008에서 진행합니다.

## 시뮬레이션 평가용 명령

`drive_route`는 `geometry_msgs/msg/Twist`를 `/cmd_vel`로 최대 20 Hz 발행합니다.
외부 VelocityControl의 body XY 제어를 사용하며 실물에는 실행하지 않습니다.
초기 odom 시각의 map→odom 역변환을 고정하고, 각 odom 시각의 map→base_footprint를
기록합니다. `run_benchmark`의 반복 실행·정답선·품질 기준은
[시뮬레이션 평가 절차](simulation-evaluation.md)에 정의합니다.

## 시작 구간 진단 인터페이스

| 토픽 | 타입 | 발행 주체·역할 |
|---|---|---|
| `/slam_toolbox/graph_visualization` | `visualization_msgs/msg/MarkerArray` | slam_toolbox의 주기적 노드·edge 시각화, 현재 약 1초 간격 |
| `/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | slam_toolbox가 처리한 scan의 추정 위치 |
| `/mapping_capture/events` | `std_msgs/msg/String` | 진단 수집기의 단계·시각·scan 수 JSON, RELIABLE/VOLATILE depth 10 |

진단 수집기는 TF를 발행하지 않습니다. `/mapping_capture/mark_slam_start`,
`/mapping_capture/mark_drive_start`, `/mapping_capture/mark_capture_end`는
`std_srvs/srv/Trigger`이며 이벤트 발행과 같은 JSON을 응답 message에 반환합니다.
단계 시각은 서비스 처리 시각이고 SLAM의 scan 수락 시각이 아닙니다.

graph 수집은 RELIABLE/VOLATILE depth 1, 센서·동적 TF 수집은 BEST_EFFORT/VOLATILE,
정적 TF 수집은 RELIABLE/TRANSIENT_LOCAL depth 100입니다.
recorder·재생은 별도 QoS YAML을 사본·해시와 보존하고, 빈 원본 QoS 메타데이터에도
`/tf_static`을 TRANSIENT_LOCAL로 재생합니다. 원본 bag은 변경하지 않습니다.
[실행 순서·시각/그래프 해석 한계·실제 결과](slam-startup-capture.md)를 참고합니다.

## 시맨틱 검출·투영 계약

| 토픽 | 타입 | 의미 |
|---|---|---|
| `/semantic/detections2d` | `vision_msgs/msg/Detection2DArray` | 원본 RGB header·픽셀 bbox, 모델 confidence |
| `/semantic/observations` | `vision_msgs/msg/Detection3DArray` | 관측 시각 map 좌표의 표면 대표점 |
| `/semantic/objects` | `vision_msgs/msg/Detection3DArray` | 최소 3회 관측한 정적 객체, TRANSIENT_LOCAL |
| `/semantic/markers` | `visualization_msgs/msg/MarkerArray` | 확정 객체 표시와 epoch 초기화 DELETEALL |
| `/semantic/status` | `std_msgs/msg/String` | 처리·거부·TF·리셋·CPU·지연 JSON |

외부 모드는 RGB/depth/CameraInfo/Detection2DArray의 frame·stamp가 모두 같아야 합니다.
최고 confidence≥0.5, 유효 bbox 중앙 50%에서 깊이를 투영합니다. 정렬된 32FC1(m)만 지원하고
최신 TF로 대체하지 않습니다. person 등 동적 클래스는 ID 없는 관측만 발행하고 저장하지 않습니다.
정적 클래스·지도 식별자·모드가 다른 JSON은 로드하지 않습니다.
자세·크기·보정된 위치 불확실성은 추정하지 않습니다.
[세부 계약·서비스](../src/mapping_semantic/README.md)와 [검출 모델 범위](detector-baseline.md)를 참고하세요.
