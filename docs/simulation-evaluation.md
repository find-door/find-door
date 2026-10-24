# 시뮬레이션 반복 평가

로봇 도착 전 `indoor_v1`의 동일 경로를 반복 주행하고 slam_toolbox 지도를
알려진 벽 면과 비교합니다. 실물 성능이나 SLAM 알고리즘 최종 선정 결과는 아닙니다.

## 실행

저장소 루트에서 빌드 후 실행합니다. 도메인 12에 기존 시뮬레이션이 없어야 합니다.
실행기가 headless 시뮬레이션을 시작하고 자신이 만든 프로세스 그룹만 종료합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/worker_following_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=12 GZ_PARTITION=codex
ros2 run mapping_eval run_benchmark \
  --output /tmp/mapping-baseline-new --runs 3 --record --confirm-simulation
```

`--params-file PATH`로 비교할 SLAM 설정을 지정할 수 있습니다.
[루프 클로저 비교 절차](loop-closure-investigation.md)를 참고하세요.
기존 출력 폴더는 덮어쓰지 않습니다. 기본 속도는 0.35 m/s, 주행 제한 시간은
회당 벽시계 240초입니다. `--confirm-simulation`은 필수이며 실물에 실행하지 않습니다.
외부 모델의 holonomic VelocityControl을 사용해 yaw 0을 유지하며 body XY 속도를
보냅니다. 실제 휠 구동·미끄러짐·실물 제어기를 검증하는 경로가 아닙니다.

`config/indoor_v1_route.yaml`의 12개 waypoint로 복도→왼쪽 방→오른쪽 방→복도를
순환합니다. 도착 허용 오차는 0.08 m입니다. scan 최소 유효 거리가 0.25 m 미만이거나
센서·clock이 정지하면 실패 처리하고 zero 명령을 보냅니다. 마지막 지점에서 2초
시뮬레이션 시간 동안 정지한 뒤 지도를 저장합니다.

## 지표와 정합

정답은 `indoor_v1_truth.yaml`의 자유공간에 접하는 벽 면 24개입니다.
외벽 바깥 면과 벽끼리 접하는 면은 제외합니다. 테스트에서 world collision과
정답선의 일치 및 경로의 벽 간격을 확인합니다.

초기 odom 타임스탬프의 `map←odom` TF를 역변환하여 map→world를 한 번 고정합니다.
이 모델의 odom은 world 기준 Gazebo 위치이며 시작점은 (1, 1)입니다.
최종 지도에 맞춰 이동·회전·스케일을 최적화하지 않습니다.

- 벽 오차: 점유 셀 중심에서 가장 가까운 유한 정답 선분까지 거리의 RMSE·P95·최댓값.
- precision: 점유 셀 중 정답선에서 0.10 m 이내인 비율.
- coverage: 정답선을 최대 0.025 m 간격으로 샘플링하여 점유 셀에서 0.10 m 이내인 비율.
- unknown 셀은 점유점으로 취급하지 않습니다. 부분 지도는 coverage가 낮아집니다.
- 궤적 참고값: odom과 같은 시각의 `map←base_footprint`를 초기 정합으로 변환한 위치 잔차.
  TF 기록 비율이 95% 미만이거나 마지막 기록이 종료 clock보다 1초 넘게 오래되면
  `trajectory_valid=false`이고 RMSE·최댓값은 null입니다.
  odom이 SLAM 입력에도 쓰이므로 **독립적인 ATE가 아닙니다**. TF 결측 수를 함께 기록합니다.

실행 전 정한 임시 기준은 P95 ≤0.15 m, precision ≥95%, coverage ≥85%입니다.
이는 회귀 비교를 시작하기 위한 기준이며 실물 합격 기준이 아닙니다.
실험 실행 성공과 품질 기준 통과를 구분합니다. 종료 코드 0은 전체 실행 완료이며,
품질 통과 여부는 manifest의 `quality_pass`를 확인합니다.

## 산출물과 재현 범위

- `manifest.json`: 실행 상태, 회별 지표, 평균·최솟값·최댓값·표본 표준편차, 품질 판정.
- `inputs/`: 사용한 프로젝트 world·SLAM 설정·경로·정답 사본. 외부 `wf_gazebo`는 복사하지 않고 해시만 기록.
- `run_XX/map.pgm`, `map.yaml`, `metrics.json`: 지도·기하 오차와 입력 해시.
- `run_XX/route_result.json`, `trajectory.csv`: 초기 고정 변환·정지 원인·궤적 잔차.
- `run_XX/bag/`: `--record` 사용 시 scan·odom·tf·tf_static·clock·cmd_vel 기록.
- `run_XX/effective_params.log`: 각 회차에 실제 적용된 SLAM 파라미터 YAML.
- `run_XX/*.log`, `roslogs/`: 인터페이스 검사·주행·저장·평가 및 ROS 로그.

bag에는 SLAM이 발행한 map→odom도 포함됩니다. 새 SLAM에 그대로 재생하면
TF 발행자가 중복될 수 있습니다. SLAM 입력만 필터링한 재생 비교는 아직 구현하지 않았습니다.
기준 커밋·미커밋 변경 여부와 실제 산출물 위치는 [evaluation.md](evaluation.md)에 남깁니다.
