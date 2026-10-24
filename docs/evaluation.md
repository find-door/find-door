# 실험·검증

## 평가 지표

| 지표 | 측정 방법 | 단위 | 단계 |
|---|---|---|---|
| 지도 정확도 | 실제 도면(또는 줄자 실측) 대비 벽 길이·복도 폭 오차 | cm | 1 |
| 위치 추정 오차 | 시뮬: Gazebo 정답 궤적 대비 위치·방향 오차 / 실물: 독립 측정한 기준 지점 대비 | cm, ° | 1 |
| 종점 폐합 오차 | 출발점 복귀 시 위치 차이, 루프 클로저 전·후 비교 (SLAM 자체 출력이라 보조 지표) | cm | 1 |
| 시맨틱 표시 정확도 | 객체가 지도에 찍힌 위치와 실제 위치 차이, 놓친 객체 수 | cm, 개 | 2 |
| 성능 | 처리 주기, Jetson CPU·GPU 사용률 | Hz, % | 전 단계 |

> 측정 절차(측정 지점 수, 반복 횟수, 정답 데이터 형식, 정답 좌표와 지도 좌표의 정합 방법)는 Phase 1 시작 시 확정 🔍

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
