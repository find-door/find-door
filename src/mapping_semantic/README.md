# mapping_semantic

`detector_mode:=external_detections`는 별도 검출기가 발행한
`/semantic/detections2d` (`vision_msgs/msg/Detection2DArray`)를 RGB·depth·CameraInfo와
4방향 exact-time 동기화합니다. 배열과 각 detection의 header도 영상의 frame/stamp와
일치해야 합니다. 검출 모델 자체는 이 mapper가 실행하지 않습니다.

외부 모드 실행: `ros2 launch mapping_semantic semantic.launch.py detector_mode:=external_detections map_id:=my_map`.
`use_sim_time:=false`는 외부 모드에서 명시적으로 허용합니다. 실물 실행 검증은 아직 하지 않았습니다.
RGB는 `rgb8`, 정렬된 depth는 미터 단위 `32FC1`만 지원합니다. `16UC1` 깊이는 변환기를
검증한 뒤 연결해야 합니다. 토픽과 map/odom/optical 프레임은 launch 인자로 지정합니다.

`static_classes` 노드 파라미터의 기본값은 `[chair, cup, bottle]`이며, 직접 실행 시
`--ros-args -p detector_mode:=external_detections -p "static_classes:=['chair','cup']"`처럼
설정합니다. 클래스 ID는 `[a-z][a-z0-9_]*`, 최대 64자이며 공백을 포함한 이름은 지원하지 않습니다.
person, bicycle, car, motorcycle, bus, train, truck, bird, cat, dog, horse, sheep, cow,
elephant, bear, zebra, giraffe는 동적 클래스로 취급하고 static_classes와 겹치면 시작을 거부합니다.
이 클래스는 `/semantic/observations`에만 ID 빈 문자열로 발행하며 지도·저장 파일에 누적하지 않습니다.
추적 ID나 움직임 추정을 제공하지 않습니다. 나머지 비허용 클래스는 무시합니다.

각 detection에서 가장 높은 hypothesis가 0.5 이상일 때만 처리합니다. 모든 confidence는
유한한 [0,1] 값이어야 합니다. bbox는 양수 크기·theta=0이어야 하며 영상 밖 box는 거부합니다.
영상 내부로 잘라낸 bbox 중앙 50% 영역에서 깊이를 투영합니다. 최소 20개·50% 이상의
유효 깊이가 필요합니다. bbox 중심은 실제 물체 중심이 아니며 출력은 **보이는 표면 대표점**입니다.
중앙 영역에도 배경이나 다른 물체가 들어오면 깊이가 잘못 연결될 수 있습니다.
관측 confidence는 입력 모델 값을 보존하고 누적 객체는 마지막 관측 값을 저장합니다.
이는 3D 위치 정확도나 융합 신뢰도 추정값이 아닙니다. 합성 fixture 모드 score=1은 색상 규칙 통과 표시입니다.

외부 지도 JSON은 schema 2에 detector_mode·static_classes·score를 저장하고 재로드 시
동일 계약을 요구합니다. fixture는 schema 1을 유지하며 기존 score/static_classes 없는 파일도
기본 fixture 계약으로 읽습니다. 모드 사이 지도 파일을 교차 로드하지 않습니다.
외부 2D 모델의 검출 성능과 RGB-D 투영의 기하 검증은 별도로 평가해야 합니다.

합성 색상 패널(`red_fixture`, `blue_fixture`)의 영상 검출 → 깊이 투영 →
관측 시각 TF → 정적 객체 병합·저장 파이프라인입니다.
기본 color_fixture 모드는 학습된 실제 물체 검출기가 아닙니다.

## 입력 계약

- `/camera/color/image_raw`: `sensor_msgs/msg/Image`, `rgb8`
- `/camera/depth/image_raw`: `sensor_msgs/msg/Image`, `32FC1`, 미터 단위, RGB에 정렬
- `/camera/camera_info`: `sensor_msgs/msg/CameraInfo`, 무왜곡 pinhole K
- 세 메시지의 해상도·`camera_color_optical_frame`·timestamp가 모두 일치해야 합니다.
- BEST_EFFORT 구독, exact time synchronization, 기본 큐 20개.
- 유효하지 않은 K, 왜곡, binning/ROI, 다른 frame·인코딩은 거부합니다.
- `/tf`의 `map→odom`은 지도 보정 감시에 사용합니다. 투영에는
  **관측 timestamp의** `map←camera_color_optical_frame` TF를 사용합니다.
  TF는 벽시계 1초까지 기다리며 최신 TF로 대체하지 않습니다.
- color_fixture 모드는 `use_sim_time=true`가 필수이며 외부 검출 모드는 명시적 false도 허용합니다.

색상 마스크는 HSV saturation≥130, value≥60, 빨강 hue≤10 또는 ≥170,
파랑 hue 100–130(OpenCV 0–179 단위)을 사용합니다. 최소 80픽셀 연결영역을
선택하고 1픽셀 침식한 마스크의 깊이를 사용합니다. 유효 깊이가 20개 미만이거나
마스크의 절반 미만이면 거부합니다. 0·NaN·inf와 0.2–10m 밖의 깊이도 거부합니다.

위치는 유효 픽셀을 각각 XYZ로 역투영한 뒤 좌표별 중앙값을 취한 **보이는 표면의
대표점**입니다. 가려진 박스 중심이나 완전한 객체 형상을 추정하지 않습니다.
외부 2D detector 연결을 위한 `bbox_mask()`와 마스크 기반 `project_mask()`는
순수 함수로 분리되어 있고 external_detections 모드에서 외부 Detection2D를 구독합니다.

## 출력

| 토픽 | 타입 | 의미 |
|---|---|---|
| `/semantic/observations` | `vision_msgs/msg/Detection3DArray` | 이번 영상의 map 좌표 관측값, 센서 timestamp |
| `/semantic/objects` | `vision_msgs/msg/Detection3DArray` | 3회 이상 관측된 누적 정적 객체, TRANSIENT_LOCAL |
| `/semantic/markers` | `visualization_msgs/msg/MarkerArray` | 확인된 객체의 구형 마커, TRANSIENT_LOCAL |
| `/semantic/status` | `std_msgs/msg/String` | JSON 상태·수신/거부/TF/리셋 카운터·CPU 시간·시뮬 시간 지연 |

누적 객체의 `Detection3D.id`는 클래스와 증가 번호를 조합한 ID입니다. fixture 모드에서는
위 합성 클래스만 사용합니다. score=1은 색상 규칙 통과 표시이며 보정된 확률이 아닙니다. bbox 크기는
추정하지 않아 0이고, 위치 covariance 0.01m²는 표시용 가정이며 실측 불확실성이 아닙니다.
자세는 추정하지 않으며 단위 quaternion과 큰 각도 covariance를 사용합니다.

같은 클래스에서 거리 0.25m 이내의 가장 가까운 객체에 관측을 병합합니다. 한 영상에서
동일 객체의 관측 횟수를 중복 증가시키지 않습니다. 3회는 서로 다른 timestamp여야 합니다.
동적 사람 추적·객체 만료는 이 정적 fixture 단계의 구현 범위가 아닙니다.

## 지도 보정 정책

고정 지도에서의 운용을 우선합니다. 온라인 SLAM 보정이 있는 경우, epoch 시작 때의
`map→odom`과 비교해 평행이동 0.2m 또는 yaw 0.1rad를 넘으면 전체 객체와 대기 관측을
지우고 DELETEALL 및 빈 객체 목록을 발행합니다. 카메라가 멈춰도 `/tf` 감시는 계속됩니다.
작은 보정도 누적해서 임계값을 넘으면 초기화합니다. 임계값 아래의 보정은 이전 관측을
재투영하지 않으므로 그 범위의 위치 오차가 남을 수 있습니다. yaw 오차는 원점에서 먼
객체에 더 큰 위치 오차를 만들 수 있습니다. 이 정책은 pose graph 기반 재투영의 대체물이
아니며, 정확한 누적 지도가 필요하면 고정 지도 모드가 필요합니다. 시뮬 시간 역행도
객체를 초기화합니다. 동일 노드 인스턴스의 새 epoch에서는 ID 번호를 재사용하지 않습니다. 새 프로세스는 번호 1부터 시작하므로 기존 이력을 이어갈 때 저장 파일을 로드합니다.

## 실행·저장·불러오기

Gazebo·SLAM은 별도로 실행합니다. 노드는 주행 명령이나 TF를 발행하지 않습니다.

```bash
export ROS_DOMAIN_ID=12 GZ_PARTITION=codex
source /opt/ros/humble/setup.bash
source ~/worker_following_ws/install/setup.bash
source install/setup.bash
ros2 launch mapping_semantic semantic.launch.py map_id:=semantic_v1 \
  output_path:=/tmp/semantic-objects.json input_path:=/tmp/semantic-objects.json
```

서비스 호출 때만 파일을 읽고 씁니다. 시작·종료 시 자동 저장·불러오기는 없습니다.
`map_id`는 동일한 지도 자산과 좌표계를 식별하도록 사용자가 명시해야 합니다.
새 지도·다른 원점·다른 세션의 지도일 때 같은 ID를 재사용하지 마세요.

```bash
ros2 service call /semantic/save std_srvs/srv/Trigger '{}'
ros2 service call /semantic/load std_srvs/srv/Trigger '{}'
```

- save: 확정 객체만 fixture schema_version=1 또는 외부 schema_version=2 JSON에 저장. 파일이 이미 있으면 실패하며
  덮어쓰지 않습니다. 부모 디렉토리는 미리 만들어야 합니다.
- load: map_id/frame/detector_mode, 객체 수치·ID, 저장·현재 보정 일치 여부 확인 후
  지도를 교체합니다. 현재 영상·TF가 확인되어야 하며 검증 실패 시 기존 지도는 유지합니다.
- 경로는 launch 인자 또는 `output_path`/`input_path` 노드 파라미터로 명시합니다.
- CameraInfo/관측 시각 검증과 map_id는 실물 외부 보정·정렬 검증을 대신하지 않습니다.

## 검증 범위

`test/test_core.py`는 ROS 없이 색상 검출·무검출, 입력 계약 거부, 깊이 무효값,
광학 축 회전, 병합·ID·최소 관측 수, 보정 리셋, JSON roundtrip·덮어쓰기 방지를 확인합니다.
실제 Gazebo 렌더링·ROS TF·저장 서비스 통합 검증 결과는 프로젝트 평가 문서에 기록합니다.

## RViz 설정

수동 화면 확인용 `config/semantic.rviz`를 설치합니다. 지도·scan·TF·객체 마커·RGB 표시를
구성했으며 YAML 구문과 토픽은 확인했습니다. RViz GUI 렌더링은 사용자 확인 항목입니다.

```bash
rviz2 -d "$(ros2 pkg prefix mapping_semantic)/share/mapping_semantic/config/semantic.rviz"
```

자동 통합 시험은 [시맨틱 검증 절차](../../docs/semantic-evaluation.md)를 따릅니다.
