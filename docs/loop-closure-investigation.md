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
중복되지 않도록 필터링해야 하며, 이 재생 도구는 아직 구현하지 않았습니다.

원자료 폴더의 `comparison.json`에 집계값, 각 `run_XX/trajectory_metrics.json`에
급변 분석 결과가 있습니다. 6개 bag 모두 6개 토픽의 메시지와 종료 메타데이터가
정상적으로 기록됐습니다. `source_snapshot/`·`provenance.json`에 실행 소스와 버전을
보존했고 `summarize.py`는 설치된 mapping_eval 환경에서 같은 집계를 재생성합니다.

구현 검증: 3개 패키지 빌드, 순수 테스트 154개 통과, colcon 171개 검사 오류·실패 0.
입력 사본 보존·실제 파라미터 수집·급변 분석은 별도 에이전트가 읽기 전용으로 검토했습니다.
