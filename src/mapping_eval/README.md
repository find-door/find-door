# mapping_eval

시뮬레이션 입력과 SLAM 출력의 인터페이스를 점검하는 `ament_python` 패키지입니다.
자동 경로 주행·지도 기하 오차·반복 실행은 [시뮬레이션 평가 절차](../../docs/simulation-evaluation.md)를 참고하세요.
궤적 잔차는 독립적인 위치 정확도 평가가 아닌 보조 지표입니다.
`analyze_trajectory`로 위치 잔차 급변을 검출하며, 실행법과 on/off 비교 조건은
[루프 클로저 조사](../../docs/loop-closure-investigation.md)에 정리합니다.

## 실행

시뮬레이션과 검사 명령은 동일한 `ROS_DOMAIN_ID`를 사용해야 합니다.
Codex 검증은 `ROS_DOMAIN_ID=12`, `GZ_PARTITION=codex`로 격리합니다.
ROS 환경과 이 워크스페이스의 `install/setup.bash`를 source한 뒤 실행합니다.

```bash
ros2 run mapping_eval check_interfaces --timeout 20
# SLAM을 실행한 상태에서 지도와 map → odom까지 검사
ros2 run mapping_eval check_interfaces --timeout 30 --require-map
```

- 종료 코드: `0` 통과, `1` 수신·TF 검증 실패 또는 중단, `2` CLI 인자 오류.
- 제한 시간은 시뮬레이션 시간이 아닌 벽시계 기준입니다. `/clock`이 없어도 종료합니다.
- `/clock` 발행자 한 개와 시각 진행, `/scan` 두 번 이상 수신, `/odom` 수신을 확인합니다. clock 진행·scan·odom 갱신은 최근 벽시계 2초 안에 있어야 합니다.
- `/scan`은 `lidar_sim_frame`, `/odom`은 `odom → base_footprint`를 요구합니다. 양수 타임스탬프와 scan의 유효 거리 측정값도 확인합니다.
- `/odom`의 header 시각은 최신 `/clock`보다 미래이거나 1초 이상 오래되면 거부합니다. 오래된 메시지를 계속 발행해도 수신 시각만으로 통과하지 않습니다.
- `/clock`이 뒤로 이동하면 이전 센서·지도 수신 상태를 초기화하고 해당 검사 실행은 실패 처리합니다. 이전 TF와 새 실행의 관측이 섞이지 않도록 시뮬 reset 후 검사 명령을 다시 실행하세요.
- 최근 시뮬 시각 1초 안의 scan 타임스탬프로 `odom → base_footprint`, `base_footprint → lidar_sim_frame` TF를 조회합니다. 최신 시각(0) 조회로 대체하지 않습니다.
- `--require-map`은 유효한 `map` 프레임 OccupancyGrid와 같은 scan 시각의 `map → odom` TF를 추가 검사합니다. 최소 한 개의 관측 셀이 필요합니다.
- 센서·clock은 BEST_EFFORT, 지도는 RELIABLE + TRANSIENT_LOCAL로 구독합니다. 지도 크기·해상도·데이터 길이·점유 값 범위를 검사합니다.

이 검사는 기본 프레임을 쓰는 실내 시뮬레이션용입니다. 모든 빔이 무한대인 열린 공간,
일시정지된 시뮬레이션, 매우 낮은 실시간 배율에서는 실패할 수 있습니다.
메시지 수신·연결 검사는 정확도 검증이나 지속적인 상태 감시를 대신하지 않습니다.
IMU·카메라, 주행 명령, 개별 TF의 중복 발행자 검사는 아직 포함하지 않습니다.

## 단위 테스트

ROS 메시지와 같은 필드를 가진 입력으로 잘못된 프레임·타임스탬프·거리·지도 데이터를 검사합니다.

```bash
PYTHONPATH=src/mapping_eval python3 -m pytest src/mapping_eval/test -q
```

통합 실행은 시뮬레이터와 SLAM이 있는 환경에서 위 CLI로 별도 검증합니다.
