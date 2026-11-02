# 문 위치·상태 ROS 연결

`door_mapper`는 기존 RGB-D·문 검출에서 출입구·문턱 후보를 구하고 관측 시각 TF로
map 좌표에 연결한다. 상태와 고정 ID는 문 메모리에서 관리한다.
이는 제한된 후보들의 ROS 연결이며 일반 환경 성능이나 실물 검증 완료를 뜻하지 않는다.
설계 계약은 [ADR-0009](adr/0009-door-ros-pipeline.md)를 따른다.

## 입력·출력

| 입력 | 타입 | 계약 |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | rgb8/bgr8 |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 정렬 32FC1 m 또는 16UC1 mm |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 무왜곡 K·동일 해상도 |
| `/semantic/detections2d` | `vision_msgs/msg/Detection2DArray` | 원본 RGB 시각·픽셀 bbox, 배열과 개별 header 일치 |
| TF | `tf2_msgs/msg/TFMessage` | 관측 시각 map←camera, map←odom 보정 감시 |

입력은 BEST_EFFORT·exact 동기화다. 최고 score door 하나를 선택하며 기본 점수 기준은 0.1이다.
문짝 bbox를 그대로 문 위치로 저장하지 않고 출입구·벽 평면에서 문턱 후보를 구한다.
복수 문 동시 처리는 아직 미구현이다. 후속 [실시간 YOLOE 어댑터](door-live.md)가 원본 묶음 입력을 제공한다.

| 출력 | 타입 | 내용 |
|---|---|---|
| `/semantic/door_map` | `std_msgs/msg/String` | JSON map_id/frame/stamp_ns/observation_stamp_ns/epoch/objects/counters/last_error |
| `/semantic/door_markers` | `visualization_msgs/msg/MarkerArray` | 위치 구와 ID·상태 텍스트, open 초록/closed 빨강/unknown 노랑 |

출력은 TRANSIENT_LOCAL이다. 현재 epoch 목록 전에 DELETEALL을 발행한다.
상태 TTL 기본 2초, TF 벽시계 대기 1초, 입력 최대 시각 지연 10초, 큐 크기 20이다.
오래된 검출 결과가 처리돼도 TTL을 넘으면 상태 표시가 unknown이다.
상태가 unknown이라고 위치의 정확도를 보장하는 것은 아니다.

## 실행

[환경 준비](setup.md) 후 외부 센서·문 검출·SLAM이 이미 발행 중일 때 연결한다.
검출기는 원본 RGB header의 Detection2DArray를 발행해야 한다. NanoDet는 문을 지원하지 않는다.

```bash
ros2 launch mapping_semantic doors.launch.py map_id:=my_fixed_map \
  output_path:=/tmp/my-doors-new.json input_path:=/tmp/my-doors-new.json
ros2 service call /semantic/doors/save std_srvs/srv/Trigger '{}'
ros2 service call /semantic/doors/load std_srvs/srv/Trigger '{}'
```

같은 지도 자산·원점·보정에 해당하는 map_id를 지정한다.
실물에서는 `use_sim_time:=false`를 지정하고 센서·프레임 인자를 확인해야 한다(실물 미검증).
map→odom은 SLAM 등이 제공하며 이 노드는 TF를 발행하지 않는다.

## 알려진 문 재관측

[ADR-0012](adr/0012-known-door-reobservation.md) 이후 reobserve_known_doors:=true가 기본이다.
검출로 먼저 확인한 출입구를 현재 RGB-D에서 재관측해 문짝 무검출 중에도 같은 ID의 상태를 갱신한다.
처음 보는 열린 문의 검출은 보완하지 않는다. [기하·TTL·복원 정책과 ON/OFF 결과](door-reobservation.md)를 따른다.

## 실제 ROS 재생 진단

저장 Gazebo RGB-D와 실제 C 검출을 닫힘 3장→열림 3장→닫힘 3장으로 재생한다.
관측 시각은 0.5초 간격으로 새로 부여하고, map→odom은 항등 변환을 제공한다.
새 TF 좌표 확인을 위한 마지막 변환 변경도 인위적이다.
이는 메시지·시각·좌표·메모리 연결 시험이며 실제 움직이는 문·LiDAR SLAM 정확도 시험은 아니다.
RViz 마커 메시지는 검사하지만 GUI 렌더링 수동 확인은 포함하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source ~/worker_following_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=12 GZ_PARTITION=codex OPENBLAS_NUM_THREADS=1
export FASTRTPS_DEFAULT_PROFILES_FILE=~/worker_following_ws/src/wf_gazebo/config/fastdds_bridge.xml
/usr/bin/python3 scripts/door_probe/ros_check.py \
  --captures experiments/door_probe/run-01 \
  --detections experiments/door_probe/eval-01/results.json \
  --output experiments/door_probe/ros-new
```

첫 두 실행에서는 대형 영상이 exact 동기화에 도착하지 않았다. 기존 외부 브리지의
SHM 16MB DDS 설정을 적용한 후 동기화·처리가 확인됐다. 외부 설정은 읽기 전용으로 사용했다.
하네스는 기존 `/clock` 발행자가 있는 도메인을 거부하고 자신이 만든 노드 프로세스 그룹만 종료한다.

## 확인 상태

2026-11-03 최종 ROS 재생 진단 `experiments/door_probe/ros-dev-06/check.json`이 complete로 끝났다.
사용한 모듈 해시는 현재 소스와 일치한다. 다음 항목을 실제 ROS 메시지·서비스로 확인했다.

- 원본 RGB-D·실제 C 검출 9쌍 처리, 동일 ID의 closed→open→closed 확정.
- MarkerArray 위치·텍스트 발행, TTL 초과 unknown 표시.
- 저장·복원 성공, 같은 파일 덮어쓰기 거부, 다른 지도 보정에서 복원 거부.
- 잘못된 검출 header 거부 1회, map→odom 0.3m 변경 시 초기화 1회.
- 시각 역행 초기화 1회. 이전 epoch와 같은 10초 시각의 새 영상에 TF를 제공하지 않으면
  과거 TF를 재사용하지 않고 `tf_timeout`으로 거부했다.
- 새 epoch의 관측 시각에 카메라 TF x를 인위적으로 0.4m 바꾸면 문 위치 출력도 같은 만큼 바뀌었다.
  이 마지막 위치 변경은 TF 사용 확인용이며 실제 카메라 이동·정확도 시험이 아니다.

초기 진단 실패 원문 `ros-dev-01`, `ros-dev-02`도 남겼다. DDS 설정을 적용한 `ros-dev-03`부터
입력 처리가 통과했고, `ros-dev-04` 이후 발견한 TF epoch 격리 문제를 고쳐 최종 진단을 수행했다.
과거 epoch TF를 쓰지 않는다는 주장은 마지막 회귀 검증을 근거로 한다.
단위 검사 전체 600개와 `mapping_semantic` colcon 검사를 수행했다.
추가 ROS 회귀 검사 `test_epoch_guard_ros.py`도 1개 통과했다.
이 검사는 동기화 callback이 timer보다 먼저 역행을 발견할 때 이미 혼합됐을 수 있는 batch를
폐기하고, callback 밖 timer에서 동기화 큐를 비우는 경로를 직접 확인한다.
원시 check.json SHA-256은 `8061aa0575850023125c0d3d6a52a1f5c2deadb03288b8f0614caf22c685ba6c`이다.

## 남은 작업

실시간 검출과 실제 LiDAR SLAM의 정지 동시 실행은 [후속 검증](door-live.md)에서 확인했다.
주행·상태 전이·독립 문/장소와 실제 촬영 자료 평가, 문짝 라벨·학습 필요성 평가,
과목 데모와 GUI 확인은 남아 있다.
