# mapping_robot

실물 설정 파일 검증, 구동 명령 없는 센서 preflight, 실물 SLAM·선택적 외부 객체 검출을 제공합니다. Gazebo와 외부 `wf_gazebo`를 의존하거나 실행하지 않습니다. 센서 드라이버·오도메트리·정적 TF는 별도로 준비해야 합니다. 실물 실행은 아직 검증하지 않았습니다.

## 프로파일

`config/robot.template.yaml`을 복사한 뒤 실제 장치에서 확인한 토픽·프레임·주기·허용 지연·거리 범위를 채웁니다. 필수 항목의 `REPLACE_ME`·`null`과 `hardware_confirmed: false`는 실행 시 거부됩니다. 비활성 카메라 항목의 `null` 예외는 아래에 설명합니다. `do_loop_closing: false`는 첫 시험의 보수적 설정이며 안정성 보증이 아닙니다. 관측 주기 기준은 벽시계 수신률입니다.

```bash
ros2 run mapping_robot validate_profile --profile /absolute/path/robot.yaml
ros2 run mapping_robot preflight --profile /absolute/path/robot.yaml \
  --duration 10 --output /tmp/robot-preflight-new.json
# RGB-D를 함께 확인할 때 --require-camera 추가
ros2 launch mapping_robot robot.launch.py profile:=/absolute/path/robot.yaml
```

`validate_profile --allow-unconfirmed`는 오프라인에서 확인 flag만 생략합니다. 미정 값은 계속 거부하고 launch/preflight에는 이 옵션이 없습니다. YAML은 schema 1의 명시된 key만 허용합니다. 토픽은 절대 이름, 프레임은 선행 `/` 없는 상대 이름이며 중복과 명령·예약 토픽을 거부합니다. 메시지 타입은 코드의 `TOPIC_TYPES`로 고정합니다: scan `sensor_msgs/msg/LaserScan`, odom `nav_msgs/msg/Odometry`, RGB/depth `sensor_msgs/msg/Image`, camera_info `sensor_msgs/msg/CameraInfo`, detections `vision_msgs/msg/Detection2DArray`.

프로파일 loader는 `mapping_robot.profile.load_profile(path)`이며 `(dict, SHA-256)`을 반환합니다. ROS 없이 호출할 수 있습니다. `validate_profile(dict)`는 오류 목록을 반환하고 파일과 모델 checksum도 검사합니다.

semantic이 비활성인 첫 SLAM에서는 optical frame과 RGB/depth/camera_info/detections 토픽을 `null`로 둘 수 있습니다. `--require-camera` 또는 semantic 활성화 시에는 실제 값이 필요합니다. `load_profile(path, require_camera=True)`로 같은 추가 검증을 요청할 수 있습니다.

## preflight 범위

5~120초 동안 최소 3개 표본, header 시각의 엄격한 증가, 최근성, scan/odom 최소 수신률, 관측기간의 70% 이상에 걸친 수신, 발행자 정확히 1개·타입을 검사하고 QoS를 기록합니다. scan 거리·각도·프레임, odom child frame·위치·단위 quaternion을 확인합니다. 무반사 `+inf`는 허용하지만 유한한 유효 거리가 하나도 없으면 거부합니다. 센서가 무효 거리를 NaN 등으로 표현한다면 검증된 어댑터가 필요합니다.

TF `odom←base`, `base←laser`는 scan header 시각으로 조회합니다. 카메라 옵션은 정해진 sync 정책으로 RGB/depth/CameraInfo를 짝짓고 RGB 시각의 `base←optical` TF를 검사합니다. 카메라 투영 전제는 `mapping_semantic`의 입력 검증을 재사용합니다. 모델 정확도·실측 외부 보정·시계 동기화 품질 자체를 증명하는 검사가 아닙니다.

실물은 `use_sim_time=false`이며 `/clock` 발행자가 있으면 거부합니다. 테스트용 `--simulation-fixture`는 `ROS_DOMAIN_ID=12 GZ_PARTITION=codex`를 강제하고 `use_sim_time=true`로 검사합니다. 이 결과는 항상 `context=simulation_fixture`, `hardware_verified=false`입니다. 느린 시뮬레이션은 벽시계 수신률 기준에서 실패할 수 있습니다. JSON은 profile 해시, context, 입력별 시각·수신률, 발행자 QoS·GID, TF 결과·오류를 기록하며 기존 파일을 덮어쓰지 않습니다. 이 패키지는 `/cmd_vel` 등 구동 명령을 발행하지 않습니다.

## 선택적 semantic

기본은 비활성입니다. 활성화하려면 `aligned_depth_confirmed: true`와 비어 있지 않은 `calibration_id`, `registration_note`로 실측 정렬 근거를 남겨야 합니다. `model_path`, 실제 파일과 일치하는 `model_sha256`, 새 `output_path`, `map_id`도 필수입니다. 초기 `static_classes`는 chair/cup/bottle의 부분집합만 허용합니다. `sync_policy`는 exact/approximate, `max_sync_skew_sec`는 0초 초과 0.1초 이하입니다. approximate는 센서 정렬을 대신하지 않습니다.

launch는 `slam_toolbox` 노드를 직접 실행하고, enabled일 때만 NanoDet와 external semantic mapper를 추가합니다. 모든 노드는 실물 시각을 사용합니다. 설정값 확인이 모델 파일의 실행 가능성이나 실제 검출 성능을 보증하지는 않습니다. 드라이버와 구동·비상정지 시험은 별도 인수 절차에서 수행해야 합니다.

## 검증

```bash
PYTHONPATH=src/mapping_robot:src/mapping_semantic python3 -m pytest src/mapping_robot/test -q
```

순수 테스트는 잘못된 profile·센서 수치·멈춘 시각·fixture 격리를 확인합니다. 빌드, launch 연결, 실제 ROS 토픽 수신·실물 검증은 별도로 수행해야 합니다.


추가 수치 검증: LaserScan의 float32 직렬화 반올림에는 최소 1µm 또는 상대 1ppm의 작은 거리 허용치를 둡니다. 카메라는 Image.data/step/encoding과 실제 CvBridge 복호 배열을 확인하며, 전영상에서 0.2~10m의 유효 depth가 최소 20개 있어야 합니다. 이 검사는 개별 검출 ROI의 깊이 품질이나 정렬 정확도를 보증하지 않습니다.

odom pose와 `odom←base` TF는 같은 odom stamp에서 평행이동 0.02m·회전 0.02rad 이내인지 비교합니다. 이 임계값은 preflight 일관성 판정값이며 센서 정확도 보증이 아닙니다. `/tf`, `/tf_static` 발행자 endpoint·GID·QoS를 기록하지만 Humble 메시지 콜백에서는 edge별 발행자 식별이 불가능하여 동일 edge의 중복 authority는 미검증입니다. `edge_authority_verified=false`를 결과에 명시합니다.

테스트용 `--synthetic-wall-fixture`는 같은 domain12/codex 격리를 요구하면서 실물처럼 `use_sim_time=false`, `/clock` 거부를 유지합니다. 결과는 `context=synthetic_wall_fixture`, `hardware_verified=false`이며 `--simulation-fixture`와 동시에 사용할 수 없습니다.
