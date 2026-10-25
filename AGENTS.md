# AGENTS.md

이 저장소에서 작업하는 에이전트의 공통 개발·협업 규칙입니다.
프로젝트 목표는 [README.md](README.md), 현재 단계는 [로드맵](docs/roadmap.md),
패키지·인터페이스 설계는 [아키텍처](docs/architecture.md)를 먼저 확인합니다.

## 기본 원칙

- 문서·커밋 메시지·답변은 한국어로 씁니다.
- 작업 전 현재 브랜치와 `git status --short`를 확인합니다. 기존 사용자 변경을 덮어쓰거나 되돌리지 않습니다.
- 현재 구현과 계획을 구분합니다. 패키지·토픽·센서 사양을 추측하지 말고 소스와 실행 결과로 확인합니다.
- 미정 사항은 `🔍`로 표시하고, 검증하지 않은 내용을 완료·통과로 기록하지 않습니다.
- 요청 범위에 필요한 변경을 우선하며, 미래 단계의 패키지나 추상화를 미리 늘리지 않습니다.

## 개발 환경과 외부 의존성

- Ubuntu 22.04 + ROS 2 Humble + Gazebo Harmonic(gz-sim 8)을 사용합니다 ([ADR-0002](docs/adr/0002-dev-environment.md)).
- `ros_gz`는 `ros-humble-ros-gzharmonic`을 사용합니다. Fortress용 `ros-humble-ros-gz*`나 Gazebo Classic 패키지를 설치 대상으로 제안하지 않습니다. Gazebo 의존 패키지를 추가할 때 충돌 여부를 확인합니다.
- `wf_gazebo`는 외부 워크스페이스 오버레이로 사용합니다 ([ADR-0003](docs/adr/0003-wf-gazebo-reuse.md)). `~/worker_following_ws/src/wf_gazebo`의 모델·센서·브리지 설정은 읽기만 하며 이 저장소에 복사하지 않습니다.
- 연구과제 비공개 자료와 대용량 rosbag, `build/`, `install/`, `log/`는 커밋하지 않습니다.
- 설치·실행 절차의 기준은 [setup.md](docs/setup.md)입니다. 저장소 루트에서 ROS 명령을 실행하기 전 환경을 준비합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/worker_following_ws/install/setup.bash
# 이 저장소를 빌드한 뒤 실행·검증할 때 추가
source install/setup.bash
```

## ROS 인터페이스 규칙

- 메시지 타입은 `sensor_msgs/msg/LaserScan`처럼 `/msg/`를 포함해 씁니다.
- 시뮬레이션 노드는 `use_sim_time:=true`를 사용하고 `/clock`은 Gazebo → ROS 단방향으로 브리지합니다.
- TF는 각 변환마다 발행 주체를 하나로 정합니다. 드라이버·융합 노드·브리지가 같은 변환을 중복 발행하지 않도록 확인합니다.
- TF 조회는 센서 메시지의 타임스탬프를 기준으로 합니다. 좌표계는 REP 105, 카메라 광학 좌표계는 REP 103 규약을 따릅니다.
- slam_toolbox의 오도메트리 입력은 `/odom` 직접 구독이 아니라 TF입니다. 실제 base 프레임과 설정을 확인하고, IMU 융합이 필요하면 별도 융합 노드를 구성합니다.
- 카메라 투영에는 `CameraInfo`, RGB·depth 동기화와 정렬, 깊이 인코딩·단위·무효값 처리가 필요합니다.
- 실물 사양과 시뮬레이션 설정값을 구분합니다. 토픽·프레임·주기·QoS 변경 시 관련 아키텍처와 실행 문서를 함께 갱신합니다.

## 검증

- 코드·설정 변경에는 해당 패키지 빌드·테스트와 변경한 동작의 실행 검증을 수행합니다.
- 문서만 변경한 경우에는 상대 링크·앵커와 기존 문서 간 일관성을 확인합니다.
- GUI 실행은 사용자에게 요청하고, 자동 검증은 `gui:=false`, `gz sim -s`, `--iterations` 등으로 수행합니다.
- 실행한 프로세스의 프로세스 그룹·PID를 추적해 해당 프로세스만 종료합니다. 다른 작업의 Gazebo·ROS 프로세스까지 일괄 종료하지 않습니다 (`pkill -f 'gz sim'` 등 이름 기준 종료 금지).
- 테스트 결과, 실제 실행 확인, 미검증 항목을 구분해서 보고합니다.

```bash
colcon build --symlink-install
colcon test
colcon test-result --verbose

# 빌드 후 이 저장소의 install/setup.bash를 source한 상태에서 실행
set -m   # 백그라운드 작업을 별도 프로세스 그룹으로
ros2 launch mapping_gazebo indoor.launch.py gui:=false &
PGID=$!
# ... 확인 ...
kill -INT -- -$PGID   # 이 launch가 띄운 gz sim·브리지까지만 종료
```

## 하위 에이전트 활용

- 서로 독립적이고 결과가 긴 조사·구현·검토 작업은 하위 에이전트에 위임할 수 있습니다. 짧은 수정과 순서 의존성이 큰 작업은 메인 에이전트가 직접 수행합니다.
- 위임할 때 목표, 필요한 배경, 담당 파일, 수정 가능 여부, 완료 조건과 반환할 결과를 명시합니다.
- 인터페이스 조사가 필요한 작업은 조사 → 규약 확정 → 독립적인 구현 분담 → 통합 검증 순서로 진행합니다. 미확정 인터페이스를 각자 추측해 병렬 구현하지 않습니다.
- 같은 파일을 여러 에이전트가 동시에 수정하지 않도록 담당 범위를 분리합니다. 공유 파일 변경이 필요하면 메인 에이전트와 조율합니다.
- 검토 담당은 읽기 전용으로 문제·영향·파일 위치·근거를 보고합니다. 중요한 문서·설계·인터페이스 변경은 작성·구현 후 별도 검토 에이전트로 교차 검증합니다.
- 하위 에이전트는 임의로 작업 범위를 확대하거나 다른 작업자의 변경을 되돌리지 않습니다. 추가 위임이 필요하면 메인 에이전트와 조율합니다.
- 한 세션 안의 브랜치 전환·커밋·병합 등 Git 상태 변경은 메인 에이전트만 조율합니다. 서로 다른 세션(Claude Code·Codex 등)이 동시에 작업할 때는 아래 [병렬 작업](#병렬-작업-claude-codecodex-동시-실행) 규칙을 따릅니다.
- 메인 에이전트는 하위 결과의 근거를 확인하고, 충돌 해결·통합·최종 검증과 사용자 보고를 책임집니다. 하위 에이전트의 완료 보고만으로 전체 작업을 완료 처리하지 않습니다.

## 병렬 작업 (Claude Code·Codex 동시 실행)

여러 에이전트 세션이 같은 PC에서 동시에 작업할 때의 규칙입니다. 원칙: **한 작업 = 한 브랜치 = 한 worktree = 한 에이전트**.

### 작업 공간

- 메인 checkout(`~/lab2/prometheus-mapping`)은 사용자와 사용자가 지정한 에이전트 하나만 씁니다. 다른 에이전트는 별도 worktree에서 작업합니다.

```bash
git fetch origin
git worktree add ../prometheus-mapping-<에이전트>-<설명> -b <type>/<설명> origin/dev
# 병합 후 정리
git worktree remove ../prometheus-mapping-<에이전트>-<설명>
```

- 다른 에이전트가 쓰는 checkout에서 브랜치 전환·`stash`·`reset`·`clean`·`checkout -- <파일>`을 하지 않습니다. `--symlink-install`의 `install/`은 `src/`를 가리키므로, 브랜치를 바꾸면 그 checkout에서 실행 중인 launch가 파일을 찾지 못합니다 (2026-10-25 확인).
- 같은 브랜치를 두 에이전트가 동시에 쓰지 않습니다. 커밋은 자기 worktree의 자기 브랜치에만 하고, force push가 필요하면 자기 브랜치에만 `--force-with-lease`로 합니다.
- 다른 PR이 `dev`에 병합되면 자기 브랜치를 `git rebase origin/dev`로 맞춘 뒤 충돌을 해결합니다.

### 빌드

- 각 worktree 안에서 `colcon build`합니다 (`build/`·`install/`·`log/`가 worktree별로 분리됨). 다른 worktree의 `install/setup.bash`를 source하지 않습니다.
- 공유 오버레이 `~/worker_following_ws`는 읽기 전용입니다. 재빌드가 필요하면 다른 에이전트의 시뮬레이션이 끝난 뒤 사용자에게 요청합니다.

### 시뮬레이션 격리

ROS 2 DDS와 Gazebo transport는 같은 PC의 프로세스끼리 토픽을 공유합니다. 에이전트마다 `ROS_DOMAIN_ID`와 `GZ_PARTITION`을 따로 쓰고, 같은 셸의 모든 ROS·`gz` 명령(launch, `ros2 topic`, `gz topic`)에 적용합니다.

| 실행 주체 | `ROS_DOMAIN_ID` | `GZ_PARTITION` |
|---|---|---|
| 사용자 (GUI·RViz2 확인) | 기본값 (0) | 설정 안 함 |
| Claude Code | `11` | `claude` |
| Codex | `12` | `codex` |

```bash
export ROS_DOMAIN_ID=11 GZ_PARTITION=claude   # Claude Code 예시
ros2 topic info /clock   # Publisher count: 1 이어야 함 (2 이상이면 다른 시뮬과 섞임)
```

- 두 world를 위 값으로 동시에 실행했을 때 토픽·스폰 위치가 섞이지 않고, 기본 도메인(0)에는 토픽이 보이지 않음을 확인했습니다 (2026-10-25).
- 종료는 [검증](#검증)대로 자기 프로세스 그룹만 합니다.

### 공유 외부 상태

- push·PR 생성·GitHub 설정·Notion 쓰기는 사용자가 그 작업에 지정한 에이전트 하나만 합니다. 지정이 없으면 하지 않고 묻습니다.
- 교차 검증 담당은 읽기 전용(Codex는 `-s read-only`)으로 실행하고 파일을 고치지 않습니다.
- 겹치기 쉬운 파일: `AGENTS.md`, `CONTRIBUTING.md`, `docs/adr/README.md`, [architecture.md 7절](docs/architecture.md#7-미결정-사항). 이 파일을 고치는 작업은 동시에 하나만 진행합니다.
- ADR 번호는 `git fetch` 후 `origin/dev`와 열린 PR(`gh pr list`)을 확인해 정하고, 브랜치 이름(`docs/adr-NNNN-...`)에 넣어 선점을 표시합니다. 번호가 겹치면 나중에 병합하는 쪽이 번호를 바꿉니다.

## Git과 문서 관리

- 브랜치·커밋·PR 규칙은 [CONTRIBUTING.md](CONTRIBUTING.md)를 따릅니다. `main`·`dev`에 직접 커밋하지 않습니다.
- 일반 작업 브랜치는 최신 `dev`, `hotfix/`는 `main`에서 만듭니다. 진행 중인 작업 브랜치나 미커밋 변경이 있으면 보존하고, 무조건 브랜치를 전환하거나 pull하지 않습니다.
- 원격과 비교하거나 새 브랜치를 준비할 때 `git remote -v`와 `git fetch`로 상태를 확인합니다.
- 커밋·병합은 사용자가 요청한 경우에만 합니다. push·PR·저장소 설정 변경은 사용자 승인 범위 안에서 수행합니다. 같은 작업 안에서 이미 받은 승인은 다시 묻지 않지만, 새 브랜치·새 PR·다른 설정 변경은 다시 확인합니다.
- 연구과제 자원의 공개 범위를 교수님께 확인하기 전에는 저장소를 Public으로 바꾸지 않습니다.
- 설계 결정은 [ADR 템플릿](docs/adr/template.md)으로 남기고 [ADR 목록](docs/adr/README.md)과 아키텍처의 미결정 사항을 함께 갱신합니다. 번호는 현재 목록을 확인해 정합니다.
- 실험 결과는 [evaluation.md](docs/evaluation.md)에 커밋·태그, rosbag·파라미터 위치, 외부 의존성 상태와 함께 기록합니다.
- 코드·설정 작업에는 관련 `docs/` 문서와 실행·검증 결과를 함께 갱신합니다. 현재 구현·계획·미검증 항목을 구분하고 파일 링크를 유지합니다.
- 사용자가 위임한 Notion 문서화는 작업 단위의 완료 조건으로 관리합니다. 지정된 메인 에이전트가 중간 결과와 최종 Git 상태를 동기화하고, 실패·미완료 상태를 완료로 표현하지 않습니다.
- Notion과 대응하는 프로젝트 문서를 변경하면 동기화 필요 여부를 알립니다. Notion 수정은 사용자가 요청하거나 승인한 범위에서 수행합니다.

## 실험 시각 자료와 Notion 동기화

- 실험 결과를 Notion에 정리할 때 실제 Gazebo 장면 캡처와 생성 지도·평가 그림을 함께 보존합니다. 사용자의 전체 바탕화면 대신 해당 창이나 별도 가상 디스플레이를 캡처합니다.
- Gazebo GUI 화면, 로봇 카메라 영상, 지도·그래프를 구분하고 world·실행 조건·검증 항목·한계를 캡션에 적습니다. 합성 표적이나 미완료 기능을 실제 인식 성과로 표현하지 않습니다.
- 생성한 이미지로 실행 화면을 대신하지 않습니다. GUI 캡처는 사용자 요청 범위에서 수행하고, 자동 검사 자체는 headless로 유지합니다.
