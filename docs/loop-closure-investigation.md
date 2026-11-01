# 루프 클로저 위치 급변 조사

[예비 실행](evaluation.md#예비-실행에서-관측한-실패)에서 약 3.57 m 위치 잔차 급변과
잘못된 벽이 관측됐습니다. 루프 클로저 경로가 원인인지 좁히기 위해
`do_loop_closing`만 바꾸는 비교 실험을 수행합니다. 기본 SLAM 설정은 유지합니다.

## 비교 조건

- 기준 코드: `c4beede` 이후 비교 도구 변경 포함. 실험별 소스 해시는 manifest에 기록합니다.
- ON/OFF 각각 새 Gazebo·SLAM 프로세스에서 3회씩, ON 3회 후 OFF 3회 순서.
- 동일 world·12개 waypoint·속도 0.35 m/s·지도 평가 기준·초기 정합.
- 양쪽 `debug_logging: true`, 조건 간 변경은 `do_loop_closing` 한 항목뿐입니다.
- 매 실행 실제 파라미터를 `effective_params.log`에 저장하고 값 일치를 확인합니다.
- 원자료: `/home/blind/lab2/prometheus-evaluation-results/2026-10-25-loop-comparison/`.
- 새 시뮬레이션 반복이므로 센서 노이즈와 처리 타이밍은 동일하지 않습니다.
  순서를 교차하지 않아 시간대별 시스템 부하의 영향을 분리할 수 없습니다.

## 재현

저장소 루트에서 ROS와 외부 오버레이·이 저장소 install을 source하고 실행합니다.
기존 폴더 대신 새 출력 경로를 사용하세요.

```bash
# 기본 설정에서 관측 옵션을 양쪽 동일하게 추가하고 loop 옵션만 다르게 생성
python3 - <<'PY'
from pathlib import Path
import yaml
base = yaml.safe_load(Path('src/mapping_slam/config/slam.yaml').read_text())
for enabled, name in [(True, 'on'), (False, 'off')]:
    parameters = base['slam_toolbox']['ros__parameters']
    parameters['debug_logging'] = True
    parameters['do_loop_closing'] = enabled
    Path('/tmp/loop_' + name + '.yaml').write_text(yaml.safe_dump(base))
PY
export ROS_DOMAIN_ID=12 GZ_PARTITION=codex
ros2 run mapping_eval run_benchmark --params-file /tmp/loop_on.yaml \
  --output /tmp/loop-on-new --runs 3 --record --confirm-simulation
ros2 run mapping_eval run_benchmark --params-file /tmp/loop_off.yaml \
  --output /tmp/loop-off-new --runs 3 --record --confirm-simulation
```

`--params-file`을 생략하면 기존 기본 SLAM 설정을 사용합니다. 지정한 파일 이름과
무관하게 실제 입력을 `inputs/slam.yaml`로 복사해 실행하고 해시를 기록합니다.

## 관측의 의미와 한계

OFF에서도 일반 scan matching과 가까운 체인 연결은 남으며 `TryCloseLoop()` 경로만
생략됩니다. OFF 결과를 오도메트리만 사용한 결과로 해석하지 않습니다.
[2.6.10 Karto 구현](https://github.com/SteveMacenski/slam_toolbox/blob/2.6.10/lib/karto_sdk/src/Mapper.cpp)

`debug_logging: true`는 Ceres 실행의 FullReport를 남깁니다. 그러나 2.6.10의
ROS logger 레벨 설정과 Karto listener 연결 때문에 모든 루프 수락 로그가
출력된다고 보장할 수 없습니다. Ceres 수렴은 대응한 장소가 옳다는 뜻도 아닙니다.
[로깅 설정](https://github.com/SteveMacenski/slam_toolbox/blob/2.6.10/src/slam_toolbox_common.cpp),
[Ceres 보고서 구현](https://github.com/SteveMacenski/slam_toolbox/blob/2.6.10/solvers/ceres_solver.cpp)

궤적 CSV에서는 추정 위치와 odom 위치의 잔차 벡터가 인접 관측 사이에 얼마나
변했는지 측정합니다. 0.2초 이하 간격에서 0.5 m 이상인 변화를 급변 후보로 표시합니다.
간격이 큰 경우에는 누락 구간으로 분리합니다. 이 값은 원본 map→odom 변환의
순수 이동량이 아니라 로봇 위치에 적용한 잔차 변화량이며, 독립적인 ATE도 아닙니다.

ON에서만 급변하면 루프 처리 연관성이 강해지지만 원인 확정에는 해당 그래프 연결
근거가 더 필요합니다. 양쪽 모두 정상이라면 실패 bag에서 SLAM 출력 TF를 제외한
동일 입력 재생 비교가 다음 과제입니다. 소수 반복의 무발생으로 안정성을 보장하지 않습니다.

분석 CLI는 CSV의 시각 순서·유한 값·필수 열을 검사하고, 기존 JSON을 덮어쓰지 않습니다.

```bash
ros2 run mapping_eval analyze_trajectory \
  --trajectory /tmp/loop-on-new/run_01/trajectory.csv \
  --output /tmp/loop-on-new/run_01/trajectory_metrics.json
```

`eligible_residual_step_max`는 긴 누락 구간을 제외한 최대 변화,
`adjacent_residual_step_max`는 누락 구간을 포함한 전체 인접 표본 최대 변화입니다.
`jump_events`에 급변 후보 시각·크기·방향을 기록합니다. CSV에 남은 관측만 분석하므로
`route_result.json`의 `trajectory_valid`·TF 누락 수와 함께 해석해야 합니다.

## 2026-10-25 결과

6회 모두 12/12 waypoint·지도 저장·임시 품질 기준을 통과했습니다.
실제 적용 파라미터를 비교했을 때 `do_loop_closing` 외에는 값이 같았고,
공통 실행 코드·world·경로·정답·외부 모델 해시도 일치했습니다.
coverage는 6회 모두 100%였고, precision은 ON 100%, OFF 99.15–99.81%였습니다.

| 조건·회차 | 벽 RMSE (cm) | 벽 P95 (cm) | 최대 인접 잔차 변화 (cm) | 급변 후보 수 | Ceres 보고서 수 |
|---|---|---|---|---|---|
| ON 1 | 2.47 | 3.95 | 2.41 | 0 | 10 |
| ON 2 | 2.96 | 6.82 | 6.92 | 0 | 10 |
| ON 3 | 2.90 | 6.39 | 6.32 | 0 | 10 |
| OFF 1 | 4.71 | 8.37 | 1.00 | 0 | 0 |
| OFF 2 | 4.11 | 5.20 | 2.00 | 0 | 0 |
| OFF 3 | 3.79 | 6.00 | 2.00 | 0 | 0 |

- 기존 실패 기록에는 분석기가 sim 50.00초의 3.566 m 급변 후보 1개를 검출했습니다.
- 신규 ON/OFF 각각 3회에는 0.5 m 이상 급변 후보가 없었습니다.
  모든 회차의 `trajectory_valid=true`이고 0.2초를 넘는 CSV 관측 간격도 없었습니다.
- ON에서는 회당 Ceres FullReport가 10개, OFF에서는 0개였습니다.
  이는 실행 관측 결과이며, 각 보고서를 특정 루프 edge 수락의 증거로 해석하지 않습니다.
- 이 표본에서는 ON의 벽 RMSE가 작았으나, 3회씩의 별도 입력 실행으로 우열이나
  일반적인 실패 확률을 판정할 수 없습니다. 상세 로깅도 최초 실패 때와 다른 관측 조건입니다.

**이번 비교에서 최초 실패는 재현되지 않았고, 원인은 미확정입니다.**
기본 `slam.yaml`의 루프 클로저 설정은 변경하지 않았습니다.
다음 조사에서는 최초 실패 bag의 `/scan`, `/clock`, odom→base 및 고정 TF를 재생하되
기록된 SLAM map→odom을 제외한 동일 입력 비교가 필요합니다. 새 SLAM과 기록 TF가
중복되지 않도록 필터링해야 하며, 당시 이 재생 도구는 아직 구현하지 않았습니다.
이후 구현·실행 결과는 [동일 입력 재생](replay-evaluation.md), 시작 전 기록과 신규
실패 관측은 [시작 구간 진단](slam-startup-capture.md)을 참고합니다.

원자료 폴더의 `comparison.json`에 집계값, 각 `run_XX/trajectory_metrics.json`에
급변 분석 결과가 있습니다. 6개 bag 모두 6개 토픽의 메시지와 종료 메타데이터가
정상적으로 기록됐습니다. `source_snapshot/`·`provenance.json`에 실행 소스와 버전을
보존했고 `summarize.py`는 설치된 mapping_eval 환경에서 같은 집계를 재생성합니다.

구현 검증: 3개 패키지 빌드, 순수 테스트 154개 통과, colcon 171개 검사 오류·실패 0.
입력 사본 보존·실제 파라미터 수집·급변 분석은 별도 에이전트가 읽기 전용으로 검토했습니다.

## 2026-10-26 원인 분석: 스캔 무게중심 기반 루프 후보 선택

[시작 구간 진단](slam-startup-capture.md#새-급변에서-확인한-사실)의 v2 실패 bag은 루프 ON 재생에서
매번 같은 실패(벽 P95 391.30 cm)를 냅니다. 이 결정적 재현 입력으로 원인을 좁혔습니다.

### 무엇이 연결됐나

- 급변 직전 scan(sim 53.6초)의 실제 위치는 **복도 (6.37, 1.04)**, 서쪽 진행입니다.
  시뮬 `/odom`은 world 원점 기준 Gazebo 자세라 정답 위치로 씁니다.
- SLAM은 이 노드(101)를 벽 너머 **Room B의 노드 60 (6.17, 4.51)**과 연결했고,
  map→odom이 y 방향 +3.67 m(x 0.045 m, yaw −0.02 rad) 보정됐습니다.

### 왜 후보가 됐나

slam_toolbox 2.6.10의 `MapperGraph::FindPossibleLoopClosure`는 후보 거리를
`GetReferencePose(use_scan_barycenter)`로 잽니다. 기본값 `true`에서는 로봇 위치가 아니라
**scan 점들의 무게중심** 사이 거리입니다
([Mapper.cpp](https://github.com/SteveMacenski/slam_toolbox/blob/2.6.10/lib/karto_sdk/src/Mapper.cpp)).
LiDAR 최대 거리 12 m에서는 문을 통해 옆 공간의 점이 들어와 무게중심이 끌립니다.

실패 bag에서 오프라인으로 계산한 값입니다 (`barycenter.py`, 정답 위치 기준):

| 비교 | 거리 | `loop_search_maximum_distance` 3.0 m |
|---|---|---|
| 노드101 scan ↔ 가장 가까운 Room B scan, 로봇 위치 | 3.48 m | 후보 아님 |
| 같은 두 scan, 무게중심 | 2.94 m | **후보** |

Room B(y≈4.5) scan 25개 중 17개의 무게중심이 3.0 m 안에 들어왔습니다.
후보가 된 뒤에는 `loop_search_space_dimension: 8.0`(±4 m) 탐색이 3.6 m 보정을 허용했습니다.

### 한 항목씩 바꾼 재생

같은 v2 bag·재생 속도 1.0·고정 초기 정합이며, 설정 차이는 표의 한 항목과 루프 최적화 횟수를
세기 위한 `debug_logging: true`뿐입니다. 루프 최적화 횟수는 `slam.log`의 Ceres `Solver Summary` 수입니다.

| 조건 | 벽 P95 (cm) | 벽 RMSE (cm) | precision | 급변 후보 | 루프 최적화 |
|---|---|---|---|---|---|
| 기준 (v2 설정 그대로) | 391.30 | 175.65 | 49.11% | 1 | 1 |
| `use_scan_barycenter: false` | 2.96 | 2.47 | 100% | 0 | 10 |
| `loop_search_maximum_distance: 2.5` | 2.92 | 2.45 | 100% | 0 | 7 |
| `loop_search_space_dimension: 4.0` | 3.14 | 2.41 | 100% | 0 | 11 |
| `loop_match_minimum_response_fine: 0.6` | 3.14 | 2.41 | 100% | 0 | 11 |

- 기준에서는 실행 전체에서 **루프 최적화가 한 번** 일어났고 그 한 번에 지도가 깨졌습니다.
- 네 조건 모두 루프를 7~11회 닫으면서 급변이 없었습니다. 루프를 끈 효과가 아니며,
  같은 bag의 loop OFF(벽 P95 6.63 cm)보다 지도가 정확합니다.
- `debug_logging` 없이 같은 5개 조건을 한 번 더 돌렸을 때 지도 지표는 위와 같았고,
  기준의 급변 후보 수만 4개로 달랐습니다 (궤적 표본 시점 차이).

### 정상 bag 회귀 확인

기존 기록 4개를 각자의 원래 설정과 `use_scan_barycenter: false`로 재생했습니다.

| bag | 원래 설정 P95 (cm) | `false` P95 (cm) | 루프 최적화 (원래 / `false`) |
|---|---|---|---|
| full-start 1 | 5.32 | 4.06 | 10 / 9 |
| loop ON 1 | **238.50** | 7.16 | 11 / 9 |
| loop ON 2 | 3.63 | 3.18 | 10 / 9 |
| loop ON 3 | 3.08 | 2.84 | 10 / 9 |

- `false`는 4개 모두 임시 품질 기준을 통과했고 P95가 같거나 작았습니다.
- **loop ON 1은 온라인에서는 통과(벽 P95 3.95 cm)했지만 원래 설정 재생에서 실패**했습니다.
  위치 잔차는 최대 0.12 m로 정상인데, 점유 셀 4236개 중 2033개가 건물 밖 y≈−1.0에 찍혔습니다.
  현재 위치가 아니라 **이미 지나온 노드들이 루프 최적화로 약 3 m 밀린** 것으로 보입니다.
  이 bag에는 그래프 진단이 없어 같은 무게중심 메커니즘인지는 🔍입니다.
- 이 실패는 현재 위치만 보는 궤적 급변 검출(`jump_event_count` 0)로는 잡히지 않습니다.
  지도 지표를 함께 봐야 합니다.

### 결론과 한계

**v2 실패의 원인은 무게중심 기반 후보 선택이 벽 너머 다른 공간의 scan을 루프 후보로 넣고,
넓은 탐색 범위가 그 잘못된 정합을 받아들인 것입니다.** 근거는 소스 확인, 오프라인 거리 계산,
한 항목씩 바꾼 결정적 재생입니다.

- 과거 예비 실패(3.566 m)는 초기 scan이 기록되지 않아 같은 원인인지 확인할 수 없습니다 🔍
- 근거는 한 world(`indoor_v1`)·재생 bag 5개입니다. 실물·다른 구조에서의 안정성을 보장하지 않습니다.
- `indoor_v1`은 복도와 방이 같은 x 범위에서 벽 하나를 사이에 두고 평행해, 이런 오연결이 일어나기 쉬운 구조입니다.
- 기본값 채택은 아래 온라인 반복 결과로 결정했습니다 ([ADR-0004](adr/0004-prehardware-slam-baseline.md#2026-10-27-루프-후보-기준-변경)).

원자료: `/home/blind/lab2/prometheus-evaluation-results/` 아래

- `2026-10-26-loop-gating/`: 5개 조건 재생(로그 없음), 조건별 `params/`, `loop_gating.sh`, `barycenter.py`·`barycenter_result.txt`
- `2026-10-26-loop-gating-debug/`: 같은 5개 조건 + `debug_logging`, `loop_gating_debug.sh`
- `2026-10-26-barycenter-regression/`: 정상 bag 4개 × 2개 설정, `regression.sh`

평가 도구가 `ROS_DOMAIN_ID=12`·`GZ_PARTITION=codex`만 허용해서, Codex가 대기 중이고
도메인 12에 다른 발행자가 없음을 확인한 뒤 그 도메인으로 실행했습니다.

### 2026-10-27 온라인 반복 확인

dev 기본 `slam.yaml`에서 `use_scan_barycenter: false`만 바꾸고(관측용 `debug_logging: true` 추가)
새 Gazebo·SLAM 프로세스로 5회 자동 주행했습니다. `--record`로 시작 전부터 bag을 기록했습니다.

| 회차 | 벽 RMSE (cm) | 벽 P95 (cm) | precision | coverage | waypoint | 급변 후보 | 루프 최적화 |
|---|---|---|---|---|---|---|---|
| 1 | 2.30 | 3.45 | 100% | 100% | 12/12 | 0 | 10 |
| 2 | 2.28 | 3.43 | 100% | 100% | 12/12 | 0 | 10 |
| 3 | 3.20 | 7.16 | 100% | 100% | 12/12 | 0 | 10 |
| 4 | 2.78 | 6.46 | 100% | 100% | 12/12 | 0 | 9 |
| 5 | 1.97 | 3.02 | 100% | 100% | 12/12 | 0 | 10 |

- 5회 모두 임시 품질 기준을 통과했고(`quality_pass: true`), 위치 최대 오차는 3.3–6.4 cm였습니다.
- 기존 온라인 루프 ON(벽 P95 3.21–6.82 cm)과 같은 범위이며, 루프를 계속 닫습니다.
- 5회 무발생이 실패 확률 0을 뜻하지는 않습니다. 원래 설정도 온라인 실행 대부분은 통과했고,
  실패는 예비 실행과 v2 두 번에서 관측됐습니다.

원자료: `/home/blind/lab2/prometheus-evaluation-results/2026-10-27-barycenter-off-online/`
(`inputs/slam.yaml`이 실제 입력 사본, `online_bary_off.sh`, 회차별 bag·지도·궤적·`trajectory_metrics.json`).

## 2026-10-31 과거 노드 이동 검사

현재 위치는 정상인데 과거 노드만 밀려 지도가 망가지는 실패는 궤적 급변 검출이 놓칩니다.
`analyze_graph`는 정답 벽 없이 `/slam_toolbox/graph_visualization`의 노드 좌표만으로
연속 스냅샷 사이 이동과 처음 관측 위치 대비 최종 이동을 계산합니다.
기준(0.5 m) 이상이면 `anomaly_candidate`로 표시합니다.

```bash
ros2 run mapping_eval analyze_graph \
  --graph run_01/diagnostics/graph.jsonl --output run_01/graph_motion.json
```

`run_benchmark --record`와 `replay_benchmark`는 자동으로 `graph_motion.json`을 남깁니다.
품질 기준(`quality_pass`)에는 넣지 않았습니다.

| 경로 | 조건 | 벽 P95 (cm) | 궤적 급변 후보 | 최대 노드 이동 (m) | 이상 후보 |
|---|---|---:|---:|---:|---|
| 재생 | v2, 원래 설정 | 391.30 | 1 | 3.437 | **있음** |
| 재생 | loop ON 1, 원래 설정 | 238.50 | **0** | 3.329 | **있음** |
| 재생 | loop ON 1, `false` | 7.16 | 0 | 0.073 | 없음 |
| 온라인 | 현재 기본값, 신규 1회 | 2.73 | 0 | 0.040 | 없음 |

- 도구 통합 전 같은 방식으로 기록한 재생 10건: 실패 2건은 3.33 m 이상, 나머지 8건(벽 P95 2.84–7.16 cm)은 0.08 m 이하.
- 저장된 온라인 기록 7건: 실패 1건(391.30 cm)만 3.511 m로 후보, 나머지 6건(3.02–7.16 cm)은 0.08 m 이하.
- 같은 v2 재생도 스냅샷 시점에 따라 최대 이동이 3.437–3.510 m로 달라집니다.

### 한계

- 시뮬레이션의 같은 world·경로 결과입니다. 실패 사례는 서로 다른 bag 2개뿐입니다.
- 시뮬 odom은 드리프트가 거의 없어 정상 보정이 작습니다. 실물에서는 올바른 루프 클로저도 노드를 크게 옮길 수 있어, 0.5 m는 잠정값이며 실물에서 다시 정해야 합니다.
- 이상 후보는 원인이나 지도 정확도 판정이 아니며, 노드 이동 없이 생기는 지도 오류는 잡지 못합니다.
- Cartographer 재생은 이 토픽이 없어 검사를 건너뜁니다 (실행 미검증).

원자료: `prometheus-evaluation-results/2026-10-31-graph-motion/`, `2026-10-30-node-motion/`

## 2026-10-31 indoor_v2 재검증

`indoor_v1` 하나에서 나온 결론을 구조가 다른 world에서 확인했습니다. `indoor_v2`는 같은 크기의 방 6개가
복도 양쪽에 있는 14 × 8.2 m 공간이며([도면](../src/mapping_gazebo/README.md#indoor_v2-도면-반복-구조-평가)),
14 waypoint·42.4 m 경로를 0.35 m/s로 주행합니다. 판정 기준은 실행 전에 정했고 결과를 본 뒤 바꾸지 않았습니다.
조건마다 `run_benchmark --world indoor_v2 --record`로 온라인 실행했고, 한 항목만 바꿨습니다.

| 조건 | 회수 | 벽 P95 (cm) | precision | coverage | waypoint | 품질 기준 | 최대 노드 이동 (m) | 이상 후보 |
|---|---:|---:|---:|---:|---|---|---:|---|
| 현재 기본값 (`use_scan_barycenter: false`) | 5 | 291.5–300.5 | 32–38% | 51–65% | 14/14 | 5회 실패 | 2.44–2.70 | 5회 |
| 원래 설정 (`use_scan_barycenter: true`) | 5 | 277.5–302.5 | 29–34% | 44–48% | 14/14 | 5회 실패 | 2.68–2.82 | 5회 |
| 루프 OFF | 3 | 12.51–13.46 | 약 90% | 95–98% | 14/14 | 3회 실패 | 0 | 0회 |

본 실험 전 시험 주행 1회(현재 기본값)도 P95 293.3 cm로 실패했고 위 표에는 넣지 않았습니다.

- 현재 기본값은 이 world에서 실패했고, `use_scan_barycenter` 값에 따른 실패 횟수 차이도 없었습니다. 이 값을 바꾼 수정은 `indoor_v1`의 결함에 대한 대응이며 잘못된 루프 연결 전반을 막지 않습니다.
- 루프 OFF에서는 큰 실패가 사라져 루프 클로저와의 관련성이 강합니다. 다만 P95 항목만 통과하고 precision이 기준(95%)에 미달해 해결책이 아닙니다.
- `analyze_graph`는 큰 실패를 모두 후보로 표시했습니다. 루프 OFF의 precision 미달은 노드가 움직이지 않아 표시하지 못했습니다.
- 시험 주행에서는 83초에 복도 (10.1, 4.04)의 로봇을 (10.31, 6.61)로 추정했고 노드 66개가 최대 2.45 m 밀렸습니다.

### 한계

- 원인은 확인하지 않았습니다 🔍. 같은 모양의 방 반복이 원인이라는 것은 아직 추정입니다.
- 일부러 잘못된 연결이 일어나기 쉽게 만든 시뮬레이션 world이며, 실물 건물에서의 실패율을 뜻하지 않습니다.
- 시뮬레이션 odom은 Gazebo 값이라 오차가 거의 없습니다.

원자료: `prometheus-evaluation-results/2026-10-31-indoor-v2/` (회차별 bag·지도·`graph_motion.json`, `params/`).

## 2026-11-01 indoor_v2 실패 후보와 설정 검증

> 2026-11-01 정정: 처음에는 아래 연결을 "잘못된 연결"로 기록했습니다. 그래프의 비연속 연결에는 루프 연결과 일반 스캔 연결이 섞여 있고, 멀리 떨어진 두 스캔의 연결이 곧 오연결도 아닙니다. 그래서 **오연결 후보**로 낮춥니다. 관측한 연결과 노드 이동은 그대로 유효합니다.

### 벽 너머 연결 후보

저장된 온라인 기록의 그래프 연결과 정답 odom을 대조했습니다(노드 시각은 스냅샷 해상도 약 1초).
실패 10회 중 9회에서 실제 거리 2 m를 넘는 첫 연결이 복도와 벽 너머 방의 평행 경로를 이었습니다.
노드 시각 추정 때문에 거리에는 최대 약 0.7 m 오차가 있을 수 있습니다.

| 실행 | 시점 | 연결 | 실제 거리 |
|---|---|---|---:|
| 현재 기본값 1, 2, 4, 5회 | 83–86초 | 복도 (약 10, 4.0) ↔ N3 (약 10.5, 6.7) | 2.66–2.69 m |
| 현재 기본값 3회 | 98.5초 | 복도 (5.0, 4.1) ↔ S2 (5.4, 1.5) | 2.61 m |
| 원래 설정 1, 2, 3, 5회 | 약 100–101초 | 복도 (약 4.1, 4.1) ↔ S1·S2 (약 4.8, 1.5) | 2.65–2.69 m |

경로가 복도(y 4.1)와 방(y 1.5, 6.7)을 약 2.6 m 간격으로 지나므로 로봇 위치 기준 거리로도
`loop_search_maximum_distance` 3.0 m 안에 들어옵니다. 그래서 `use_scan_barycenter`로는 막히지 않을 수 있습니다.
같은 시점의 노드 이동(2.4–2.8 m)과 후보 거리를 줄였을 때 실패가 사라진 결과는 이 가설을 지지하지만, 연결의 생성 경로와 상대 변환을 확인하지 못해 입증하지는 못했습니다 🔍.
원래 설정 4회는 복도끼리의 2.06 m 연결이 먼저 나타나 형태가 다르며 확정하지 않았습니다 🔍.

### 같은 bag 재생 비교

현재 기본값 온라인 1회차·3회차 bag에서 한 항목씩 바꿨습니다.

| 변경 항목 | 1회차 P95 / precision | 3회차 P95 / precision |
|---|---:|---:|
| 기준선 | 296.2 cm / 37.6% | 304.5 cm / 35.2% |
| `loop_search_maximum_distance: 2.0` | 8.46 cm / 100% | 13.93 cm / 83.6% |
| `loop_search_space_dimension: 4.0` | 8.30 cm / 100% | 13.37 cm / 86.5% |
| `loop_match_minimum_response_fine: 0.6` | 8.33 cm / 100% | 13.50 cm / 82.9% |
| `loop_match_minimum_chain_size: 20` | 303.2 cm / 40.5% | 302.5 cm / 38.8% |
| `ceres_loss_function: HuberLoss` | 286.5 cm / 30.2% | 302.5 cm / 30.9% |

### 온라인 후보 검증

기준은 미리 정했습니다: 두 world 모두 5회 전부 임시 품질 기준 통과.

| 후보 | `indoor_v1` | `indoor_v2` | `indoor_v2` P95 | `indoor_v2` precision |
|---|---|---|---:|---:|
| `loop_search_maximum_distance: 2.0` | 5/5 | 3/5 | 8.67–13.50 cm | 82.7–100% |
| `loop_match_minimum_response_fine: 0.6` | 5/5 | 1/5 | 8.62–13.75 cm | 92.2–98.0% |
| `loop_search_space_dimension: 4.0` | 5/5 | 0/5 | 13.66–13.93 cm | 82.6–93.5% |

**기본값은 유지합니다.** 후보의 완주한 `indoor_v2` 실행에서는 큰 실패가 없었고(최대 노드 이동 0.09 m 이하)
현재 기본값(약 300 cm)보다 훨씬 나았지만 미리 정한 기준을 통과하지 못했습니다.

- `space-4.0`의 `indoor_v2` 5회만 `ROS_LOCALHOST_ONLY=1`, `GZ_IP=127.0.0.1`로 실행했습니다. 다른 조건과 통신 방식이 달라 직접 비교에 주의합니다. 일반 네트워크 재실행 1회(P95 9.03 cm, precision 95.6%)는 집계에서 뺐습니다.
- Wi-Fi 재연결로 2회 주행이 `/scan` 신선도 검사에서 중단됐습니다. 전체 42회 중 이 2회에만 `Network is unreachable`이 기록됐고, 그중 한 시각은 NetworkManager 기록과 일치합니다. 이 2회도 집계에서 뺐습니다.

### 한계와 남은 문제

- 재생 기록에 연결 정보를 남기지 않아, 후보별로 해당 연결이 사라졌는지 직접 확인하지 못했습니다 🔍. 매칭 점수가 왜 기준을 넘었는지도 확인하지 않았습니다 🔍.
- Wi-Fi 끊김과 `ros2 param dump` 시간 초과 같은 실행 중단은 인프라 실패이며 지도 품질 결과로 세지 않습니다.
- 후보 실행은 위치 오차가 최대 11–22 cm까지 쌓이고, 평가 허용 오차가 10 cm라 precision이 떨어지는 것으로 보입니다. 원인은 미확인입니다 🔍.
- 후보 값이 `indoor_v2` 경로에 맞춰졌을 수 있습니다. 벽 너머 경로가 2 m보다 가까우면 `maxdist-2.0`도 같은 착각을 막지 못할 수 있습니다.
- 5회씩의 차이라 후보 사이 우열은 단정하지 않습니다.

원자료: `prometheus-evaluation-results/2026-11-01-indoor-v2-cause/`, `2026-11-01-candidate-validation/`

### 스캔 매칭과 처리 간격

오차가 가장 컸던 bag(`maxdist-2.0` 4회차)을 재생하며 한 항목씩 바꿨습니다.

| 조건 | 벽 P95 (cm) | precision | coverage | 위치 오차 최대 (cm) |
|---|---:|---:|---:|---:|
| 기준 (`maxdist-2.0`) | 13.04 | 93.3% | 97.9% | 24.1 |
| `use_scan_matching: false` | 3.50 | 100% | 100% | 0.0 |
| `minimum_travel_distance`·`heading` 0.5 | 5.84 | 100% | 100% | 4.0 |
| `fine_search_angle_offset: 0.00175` | 12.10 | 93.8% | 99.2% | 15.5 |
| `angle_variance_penalty: 10.0` | 13.04 | 93.3% | 97.9% | 24.1 |
| `distance_variance_penalty: 10.0` | 13.37 | 82.9% | 90.6% | 22.7 |

- 스캔 매칭을 끄면 오차가 사라져 벽 어긋남은 스캔 매칭 단계에서 생깁니다. 다만 이때는 정답 odom을 그대로 쓰므로 SLAM 성능 결과가 아닙니다.
- `map → odom` yaw가 약 1° 틀어지고 0.2° 단위로 변하며, 처음 80초 안에 위치 오차가 쌓입니다. 왜 한쪽으로 틀어지는지는 미확인입니다 🔍.

처리 간격 0.5 m·0.5 rad 온라인 검증(loopback 통신, 지도 품질 기준):

| 조건 | `indoor_v1` | `indoor_v2` | 벽 P95 (cm) | 최대 노드 이동 |
|---|---|---|---:|---:|
| `maxdist-2.0` + 간격 0.5 | 5/5 | 5/5 | 2.97–7.99 | 0 m |
| 간격 0.5 단독 | 5/5 | 1회 완료·통과, 2회차 인프라 중단 | 2.61–5.34 | 0 m |

- 지도 점수는 통과했지만 **후보로 채택하지 않습니다.** 노드 이동 0 m는 관측된 보정이 없다는 뜻이며, 루프 시도나 수락이 없었다는 증거는 아닙니다. 시뮬레이션 odom은 오차가 거의 없어 보정 능력을 평가할 수도 없습니다.
- 재생에서 루프 보정 이전 구간(80초 전)의 위치 오차도 24.06 cm에서 4.00 cm로 줄어, 순차 정합 자체가 나아진 효과가 있습니다.
- 연속 노드 간격 중앙값이 0.14 m에서 0.97 m로 벌어졌습니다. `loop_match_minimum_chain_size: 10`이 차지하는 거리도 함께 달라졌을 수 있습니다 🔍.
- 그래프 시각화만으로는 루프 시도·수락을 구분할 수 없고, `debug_logging`의 `Solver Summary`도 최적화 실행 횟수일 뿐입니다.

### 현재 입장

- `slam.yaml`은 **비교 기준**이며 권장 설정이 아닙니다. `indoor_v2`에서 약 300 cm로 실패하는 설정입니다.
- 실물 기본값은 미선정입니다. 실물 첫 시험 프로파일의 `do_loop_closing: false`는 그대로입니다.
- 원자료: `prometheus-evaluation-results/2026-11-01-pose-drift/`, `2026-11-01-travel-validation/`
