# 공개 객체 검출 기준선: NanoDet CPU

NanoDet-m-plus-1.5x_416의 COCO80 ONNX 모델을 OpenCV DNN CPU로 실행하는 독립 2D 검출기입니다. 입력 영상의 header와 원본 픽셀 좌표를 유지한 `vision_msgs/msg/Detection2DArray`를 발행합니다. **이 단계는 실제 RGB 영상의 학습 모델 추론이며, 깊이 투영·객체 지도 병합·실물 정확도 검증과는 별개입니다.** 기존 색상 fixture 검증은 그대로 구분합니다.

## 모델과 출처

상류 구현·모델: [OpenCV Zoo NanoDet](https://github.com/opencv/opencv_zoo/tree/main/models/object_detection_nanodet). 사용 대상은 `object_detection_nanodet_2022nov.onnx`이며 int8 변형은 이 기준선의 검증 대상이 아닙니다. 학습 모델·상류 Python 코드의 공개 라이선스는 해당 디렉터리의 Apache-2.0 LICENSE를 따릅니다.

- 로컬 확인 모델 SHA-256: `4b82da9944b88577175ee23a459dce2e26e6e4be573def65b1055dc2d9720186`.
- 상류 `nanodet.py` SHA-256: `5ba84922eaefdc48d0066cffdfb721d0d434c5f3b92213563161e00da26cea1b`.
- 코드는 [third_party/nanodet.py](../src/mapping_semantic/mapping_semantic/third_party/nanodet.py)에 원본 그대로 보관합니다. [LICENSE](../src/mapping_semantic/mapping_semantic/third_party/LICENSE)와 [NOTICE](../src/mapping_semantic/mapping_semantic/third_party/NOTICE)에 출처·변경 관계를 기록했습니다. 다운로드 시점 upstream commit은 미기록이며 파일 해시로 식별합니다.
- 모델 가중치는 저장소에 포함하지 않습니다. 모델 경로는 반드시 사용자가 지정하며, 없거나 로딩할 수 없으면 명확히 실패합니다. 색상 fixture로 대체하지 않습니다.
- 실행마다 모델 해시를 계산해 JSON·상태에 남깁니다. `expected_sha256`을 지정하면 일치 여부도 강제합니다.

COCO80에는 `person`, `chair`, `bottle` 등은 있지만 **문·계단·소화기 클래스는 없습니다.** `fire hydrant`는 소화기가 아닙니다. 원하는 실물 대상의 클래스 지원 여부·평가셋·추가 학습 필요성은 별도로 결정해야 합니다. 상류 성능 수치를 이 프로젝트의 측정값으로 취급하지 않습니다.

## 전처리·출력 계약

RGB를 종횡비를 유지하여 416×416에 letterbox하고, 정수 리사이즈의 실제 x/y 배율과 padding을 기록합니다. OpenCV Zoo의 BGR 평균·표준편차에 맞춰 채널을 변환합니다. 출력은 안정적인 softmax와 클래스별 NMS를 거친 뒤 padding·배율을 역변환하여 원본 영상 범위로 clip합니다. padding에만 존재하는 0 면적 박스는 제거합니다. OpenCV 4.5.x에서 NMS index가 Nx1 또는 1D로 반환되는 경우를 모두 처리합니다.

| 항목 | 계약 |
|---|---|
| 입력 | `/camera/color/image_raw`, `sensor_msgs/msg/Image`, rgb8 또는 bgr8 |
| 입력 QoS | best_effort / volatile / depth 1 |
| 출력 | `/semantic/detections2d`, `vision_msgs/msg/Detection2DArray`, reliable / volatile / depth 1 |
| 좌표·시간 | 입력 header 그대로, bbox는 원본 RGB 픽셀의 중심·가로·세로 |
| 클래스·score | COCO 클래스 문자열, 모델 confidence; 보정된 확률로 해석하지 않음 |
| 식별·3D pose | 추적 ID를 생성하지 않음. Detection2D hypothesis의 기본 pose는 미추정이며 사용하지 않음 |
| 상태 | `/semantic/detector_status`, `std_msgs/msg/String` JSON, reliable / volatile / depth 1 |
| 실행 방식 | CPU, 기본 5 Hz 추론 타이머, 최신 대기 영상 1장으로 교체 |

입력 수신과 추론은 별도 callback group으로 실행하며 추론 작업은 동시에 하나만 수행합니다. 추론 타이머 설정은 기본 5 Hz이며 실제 처리율이나 완료 메시지 간격을 보장하지 않습니다. 상태에는 모델 checksum, OpenCV 버전, 입력·교체·실패·처리 수, 최근 처리·큐 대기 wall time과 입력 시각 대비 지연을 기록합니다. 시뮬레이션은 `use_sim_time=true`로 실행해야 센서 시각 지연을 해석할 수 있습니다. 콜백 이전 DDS 대기 시간은 큐 대기 wall time에 포함되지 않습니다.

## 오프라인 실행

빌드·환경 준비는 [setup.md](setup.md)를 따릅니다. 아래는 실행 예시이며 모델과 입력 영상은 저장소 밖에 둡니다. 출력 JSON은 새 파일만 허용합니다.

```bash
ros2 run mapping_semantic nanodet_detect \
  --image /absolute/path/image.jpg \
  --model /absolute/path/object_detection_nanodet_2022nov.onnx \
  --expected-sha256 4b82da9944b88577175ee23a459dce2e26e6e4be573def65b1055dc2d9720186 \
  --output /tmp/nanodet-result-new.json
```

설치 전 순수 Python 진입점은 `PYTHONPATH=src/mapping_semantic python3 -m mapping_semantic.nanodet_detector`입니다. JSON에는 모델 출처 식별 정보, 입력 크기, 처리 시간, 클래스·score·원본 xyxy 박스를 저장합니다. 단일 프레임 시간에는 초기 warm-up 영향이 있을 수 있으므로 처리율 벤치마크로 사용하지 않습니다.

## ROS 실행

시뮬레이터·SLAM과 동일한 격리 도메인을 사용하고, 이미 source한 자기 worktree에서 실행합니다.

```bash
ros2 run mapping_semantic detector_node --ros-args \
  -p use_sim_time:=true \
  -p model_path:=/absolute/path/object_detection_nanodet_2022nov.onnx \
  -p expected_sha256:=4b82da9944b88577175ee23a459dce2e26e6e4be573def65b1055dc2d9720186 \
  -p max_hz:=5.0
```

검출 노드는 `/semantic/detections2d`만 담당합니다. `semantic.launch.py detector_mode:=external_detections`로 RGB-D 투영 mapper를 연결합니다. RGB/depth/CameraInfo/검출의 exact header, 정렬·관측 시각 TF를 검사하며 person은 정적 지도에 저장하지 않습니다. 임의의 2D bbox 중심을 3D 객체 중심으로 간주하지 않습니다.

## 검증 범위

모델 다운로드 없이 실행되는 단위검사는 종횡비·역변환·clip·padding 박스 제거·잘못된 영상/출력 거부·class-aware NMS index 형식·stable softmax·빈 검출·모델 경로/해시 검증을 포함합니다. 실제 ONNX 양성 추론, ROS 헤더·bbox 보존, 검출 빈 결과·잘못된 인코딩 거부·입력 중단, 합성 깊이를 사용한 전체 연결을 실행했습니다. [시맨틱 검증](semantic-evaluation.md)에 입력·결과·한계를 기록합니다. 실물 도메인 정확도, Jetson 지속 성능과 대상 클래스 선정은 아직 검증하지 않았습니다.
