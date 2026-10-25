# 실험·검증

## 평가 지표

| 지표 | 측정 방법 | 단위 | 단계 |
|---|---|---|---|
| 지도 정확도 | 실제 도면(또는 줄자 실측) 대비 벽 길이·복도 폭 오차 | cm | 1 |
| 위치 추정 오차 | 시뮬: Gazebo 정답 궤적 대비 위치·방향 오차 / 실물: 독립 측정한 기준 지점 대비 | cm, ° | 1 |
| 종점 폐합 오차 | 출발점 복귀 시 위치 차이, 루프 클로저 전·후 비교 (SLAM 자체 출력이라 보조 지표) | cm | 1 |
| 시맨틱 표시 정확도 | 객체가 지도에 찍힌 위치와 실제 위치 차이, 놓친 객체 수 | cm, 개 | 2 |
| 성능 | 처리 주기, Jetson CPU·GPU 사용률 | Hz, % | 전 단계 |

> 시뮬레이션 지도 기하 평가는 [반복 평가 절차](simulation-evaluation.md)로 정의했습니다.
> 실물 측정 절차와 독립적인 위치 정답 수집은 미정 🔍

## 결과 기록

| 날짜 | 환경 (시뮬/실물·장소) | 실험 | 기대값 | 측정값 | 결과 | 커밋/태그 |
|---|---|---|---|---|---|---|
| | | | | | | |

> 재현성을 위해 결과마다 사용한 커밋 해시(또는 태그)와 rosbag/파라미터 파일 위치를 남깁니다.

## 지도 캡처

실험별 생성 지도와 실제 도면을 나란히 붙입니다. 이미지는 `docs/images/`에 둡니다.

## 초기 패키지 연결 검증 — 2026-10-25

- 기준 커밋: `d58be6f` (`feat/indoor-world`), 작업 브랜치: `feat/project-structure`의 미커밋 변경 포함.
- 환경: Ubuntu 22.04 / ROS 2 Humble / Gazebo Harmonic, slam_toolbox 2.6.10.
- world: `src/mapping_gazebo/worlds/indoor_v1.sdf`, SLAM 설정: `src/mapping_slam/config/slam.yaml`.
- 격리: `ROS_DOMAIN_ID=12`, `GZ_PARTITION=codex`, GUI 없이 기본 스폰 위치에서 정지 상태.
- 이 실험에는 rosbag을 기록하지 않았습니다. 정확도·주행·루프 클로저 성능 결과가 아닙니다.

| 검사 | 결과 |
|---|---|
| `colcon build --symlink-install` | 3개 패키지 빌드 통과 |
| `mapping_eval` 순수 단위 테스트 | 34개 통과 (frame·scan·map·odom 시각 오류 입력 포함) |
| colcon 테스트 결과 | 오류·실패 0, launch 린트·CMake·XML 검사 포함 |
| 시뮬 없는 격리 도메인에서 `check_interfaces --timeout 2` | 시간 초과, 종료 코드 1 확인 |
| 통합 launch + `check_interfaces --timeout 60 --require-map` | 센서·시간 진행·TF·지도 검사 통과 |
| `/scan`, `/clock` 발행자·QoS | 각각 발행자 1, RELIABLE/VOLATILE 확인 |
| `map_saver_cli` 저장 | PGM + YAML 생성, 201×122셀, 해상도 0.05 m |
| SLAM 종료 후 `map_server` configure/activate | 저장 지도 수신, 동일 크기·해상도 확인 |
| 독립 에이전트 검토 | 오래된 odom 반복 발행 오통과 가능성 수정, 최종 재검토 차단 사항 없음 |

XML 스키마 검사와 ROS DDS 실행은 샌드박스의 네트워크·소켓 제한 때문에
권한 승인 후 재실행했습니다. 최초 재로딩 검사에서는 검사 프로세스에 도메인 설정이
누락되어 서비스 검색이 실패했으며, 동일 도메인을 적용한 재검사에서 통과했습니다.

로컬 로그·지도: `/tmp/prometheus-structure-validation/` (임시 산출물, 저장소에 포함하지 않음).
재현 명령은 [mapping_slam README](../src/mapping_slam/README.md)와
[mapping_eval README](../src/mapping_eval/README.md)를 참고합니다.

외부 의존성은 Git으로 관리되지 않아 이번에 읽은 주요 파일의 SHA-256을 기록합니다.

| `wf_gazebo` 상대 경로 | SHA-256 |
|---|---|
| `launch/spawn_robot.launch.py` | `279cfceeecb1f8fb40b42a1cf0ea32e46fcc91da0256b09d1c2e1d68ccf8fa6e` |
| `config/gz_bridge.yaml` | `7b984fc32d60f82f84a9b8819541fc86a450b49abfe33380d9970322b3816735` |
| `urdf/rosorin.gz.xacro` | `13f4aea542a248dc5e8830bcda0017100f393adc12a1402b8e74d3a6dcd13b74` |
| `urdf/mecanum.xacro` | `c42d801f711b05f74a5da30b5afc868b49b604fb37995cb43c11317ce6cf5346` |

## 자동 주행·지도 기하 평가 — 2026-10-25

- 실행 당시 기준 커밋: `262b838`, 미커밋 변경 포함. 이후 구현·기록을 `c4beede`로 커밋했습니다.
- 환경: ROS 2 Humble / libgz-sim8 8.15.0 / slam_toolbox 2.6.10 / nav2_map_server 1.1.20.
- 경로·정답·고정 초기 정합·임시 품질 기준: [평가 절차](simulation-evaluation.md).
- 실행: `run_benchmark --runs 3 --record --confirm-simulation`, 속도 0.35 m/s, headless.
- 출력: `/home/blind/lab2/prometheus-evaluation-results/2026-10-25-baseline-v2/`.
  `manifest.json`에 설정·외부 모델·평가 코드 해시와 회별 원자료가 있습니다.
  `inputs/`에 실제 사용한 world·SLAM 설정·경로·정답을 보존했고,
  `source_snapshot/`에 프로젝트 소스, `provenance.json`에 기준 커밋·패키지 버전을 보존했습니다.
  3회차의 실제 ROS 파라미터는 `run_03/slam_toolbox.yaml`에 별도 저장했습니다.

| 회차 | waypoint | 벽 RMSE | 벽 P95 | precision (10 cm) | coverage (10 cm) | 임시 기준 |
|---|---|---|---|---|---|---|
| 1 | 12/12 | 3.71 cm | 6.28 cm | 100.0% | 100.0% | 통과 |
| 2 | 12/12 | 2.36 cm | 3.46 cm | 100.0% | 100.0% | 통과 |
| 3 | 12/12 | 2.55 cm | 3.14 cm | 100.0% | 100.0% | 통과 |

3회 벽 RMSE 평균은 2.87 cm, 표본 표준편차는 0.73 cm입니다.
P95 평균은 4.29 cm, 표본 표준편차는 1.73 cm입니다.
세 실행 모두 궤적 기록 유효 조건을 충족했고 TF 누락은 각각 1·2·2개였습니다.
궤적 잔차는 SLAM 입력과 공유하는 Gazebo odom 기준이므로 독립적인 ATE가 아닙니다.

### 예비 실행에서 관측한 실패

정식 3회 이전 예비 실행도 삭제하지 않고
`/home/blind/lab2/prometheus-evaluation-results/2026-10-25-baseline/`에 보존했습니다.
주행·지도 저장·bag 기록은 성공했으나 평가 CLI가 음수 지수 표기를 읽지 못해 실행기가
실패했습니다. 인자를 고정 소수점으로 전달하도록 수정한 뒤, 저장된 지도는 별도로
평가하여 `metrics-recovered.json`에 남겼습니다. 정식 3회 통계에는 포함하지 않습니다.

이 예비 지도는 벽 RMSE 167.49 cm, precision 약 50.0%, coverage 약 98.6%로 실패했습니다.
시뮬 49.95초까지 궤적 잔차 RMSE는 1.47 cm였으나 50.00초에서 y 잔차가 약 3.57 m
급변했습니다. bag의 원본 map→odom TF에서도 같은 변화가 확인되며,
Ceres 실행 중 스레드 수 상한 경고가 출력된 시점과 인접해 발생했습니다.
이 경고는 최적화 성공이나 루프 수락을 입증하는 로그는 아닙니다. 반복되는 벽 구조의 잘못된 루프 클로저가
유력한 가설이지만, 루프 수락 로그나 on/off 비교가 없어 아직 확정하지 않습니다.
설정 변경 없이도 정식 반복 3회에서는 정상 결과가 나왔으므로 **안정성 확보 완료로 해석하지 않습니다**.

![예비 실패 지도와 정식 첫 반복 지도, 고정 초기 정합](images/simulation-baseline-2026-10-25.png)

### 검증과 다음 과제

- 3개 패키지 빌드 통과, 순수 단위 테스트 98개 통과, colcon 오류·실패 0.
- 에이전트가 구현을 분담하고 별도 읽기 전용 검토를 수행했습니다. 프로세스 그룹 종료,
  실행 입력 고정, TF 누락에 따른 궤적 유효성 처리를 보완했습니다.
- 정식 3회 모두 인터페이스 검사·전체 경로·지도 저장·평가·rosbag 기록 완료.
- 다음 과제: 같은 조건에서 루프 클로저 on/off 및 그래프 연결 로그를 비교하여 예비 실패 원인 확인.
- 실물 주행, 휠 오도메트리, 독립적인 위치 정확도, 카메라 융합, 알고리즘 비교는 미검증입니다.

## 루프 클로저 ON/OFF 비교 — 2026-10-25

`c4beede` 이후 비교 도구의 미커밋 변경으로 ON/OFF 각 3회 실행했습니다.
실제 파라미터는 `do_loop_closing`만 다르며 양쪽 상세 로깅을 켰습니다.
6회 모두 지도 임시 기준 통과, 0.5 m 이상 위치 잔차 급변 후보는 0개였습니다.
최초 예비 실패 기록에서는 같은 분석기가 3.566 m 급변을 검출했습니다.

**실패를 재현하지 못했으므로 원인은 미확정이며 기본 설정은 유지합니다.**
회별 수치·해석·재현 명령은 [루프 클로저 조사](loop-closure-investigation.md)에 정리했습니다.
원자료는 `/home/blind/lab2/prometheus-evaluation-results/2026-10-25-loop-comparison/`에 있습니다.

## 로봇 도착 전 소프트웨어 준비 — 2026-10-25

기준 dev `0add5f0`, `feat/prehardware-readiness`의 변경으로 실행했습니다.
동일 기록 입력의 slam_toolbox·Cartographer 각각 3회와 OFF 1회 재생을 완료했습니다.
지도 지표는 [재생 결과](replay-evaluation.md), 실제 RGB-D의 두 표적 이동·ID·저장·복원과
학습 검출/외부 투영의 구분은 [시맨틱 결과](semantic-evaluation.md)에 기록했습니다.
최초 온라인 급변은 재생에서 재현되지 않았으며 초기 scan 전체가 기록되지 않은 한계가 있습니다.
4개 패키지 빌드, 순수 테스트 285개, colcon 303개 검사 오류·실패 0.
실물 검증은 [로봇 인수](robot-acceptance.md)에서 별도 수행합니다.

재생·시맨틱 소프트웨어 구현은 [PR #7](https://github.com/reconmap-lab/prometheus-mapping/pull/7), dev `f0b37c0`에 병합했고 원격 CI를 통과했습니다.
