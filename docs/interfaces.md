# 초기 인터페이스 계약

이 계약은 `mapping_gazebo` → `mapping_slam` → `mapping_eval`의 시뮬레이션 기준입니다.
실물 센서와 시맨틱 객체 메시지는 아직 미정입니다. 표준 ROS 메시지로 충분한 현재 단계에서는
별도 `mapping_interfaces` 메시지 패키지를 만들지 않습니다.

## 패키지 경계

| 패키지 | 소유하는 것 | 소유하지 않는 것 |
|---|---|---|
| `mapping_gazebo` | 실내 world, 외부 launch wrapper | 로봇 모델·센서·브리지 구현 |
| `mapping_slam` | SLAM 설정, 단독·통합 launch | 센서 드라이버, odom·고정 TF 발행 |
| `mapping_eval` | 입력 계약 검사 CLI | 로봇 제어, SLAM 알고리즘, 지도 정확도 평가(후속 작업) |
| `mapping_semantic` (예정) | 검출·깊이 투영·객체 지도 | 현재는 패키지·커스텀 메시지를 생성하지 않음 |

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
카메라 인코딩·정렬·무효 깊이는 Phase 2에서 런타임 확인합니다. NaN·inf·0 등 무효 깊이를 투영하지 않습니다.

## TF와 시간

| 변환 | 유일한 발행 주체 |
|---|---|
| `map → odom` | slam_toolbox |
| `odom → base_footprint` | Gazebo OdometryPublisher → 외부 브리지 |
| `base_footprint → base_link → 센서` | 외부 launch의 robot_state_publisher |

외부 launch가 Gazebo·스폰·브리지·robot_state_publisher를 실행합니다.
이 프로젝트에서 같은 노드를 추가로 실행하지 않습니다. SLAM의 odom 입력은 TF입니다.
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
