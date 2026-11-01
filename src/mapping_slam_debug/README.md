# mapping_slam_debug

slam_toolbox 2.6.10의 수락된 루프 클로저를 기록하는 시뮬레이션 분석용 솔버 플러그인입니다. 계산은 `solver_plugins::CeresSolver`에 그대로 넘기고, 최적화 호출과 직전 연결만 JSONL로 남깁니다. slam_toolbox 본체와 실물 프로파일은 바꾸지 않습니다.

실험용 SLAM YAML 사본에 다음을 추가합니다.

```yaml
solver_plugin: mapping_slam_debug::LoggingSolver
loop_log_path: /absolute/path/loops.jsonl
```

기록은 [`analyze_loops`](../mapping_eval/mapping_eval/loop_events.py)로 시뮬레이션 정답 odom의 상대 변환과 비교합니다.

```bash
ros2 run mapping_eval analyze_loops --log loops.jsonl --output loops.json
```

## 한계

- 거절된 시도와 coarse·fine 점수는 기록하지 않습니다.
- 연결 생성 경로가 기록에 없어 최적화 직전 마지막 새 연결을 루프 연결로 추정합니다. 새 연결이 없거나 연속 노드의 순차 연결이면 `unknown`으로 셉니다.
- 정답 비교는 odom이 정답인 시뮬레이션에서만 의미가 있습니다.
- 현재 CI는 Python만 검사하므로 이 패키지는 로컬 `colcon build`로 확인합니다.
