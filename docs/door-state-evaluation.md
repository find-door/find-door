# 깊이 기반 문 상태 후보의 개발 진단

2026-11-02 후속 구현 승인을 받아 [계획](door-next-stage-plan.md)의 깊이 판단 후보를 구현했다.
현재 결과는 **정답 출입구 ROI를 제공한 개발 진단**이다. 검출부터 지도 표시까지의 전체 성능,
독립 시험셋 성능이나 실물 결과로 해석하지 않는다.

## 현재 구현

[door_state.py](../src/mapping_semantic/mapping_semantic/door_state.py)의
`estimate_state(depth_m, K, doorway_bbox, config=None)`은 정렬된 미터 깊이,
무왜곡 pinhole K와 영상 내부 출입구 xyxy bbox를 받는다.
문짝 bbox는 출입구 전체를 포함하지 않을 수 있으므로 입력 의미가 다르다.
반환은 `state`, `reason`, `evidence`, `config`의 JSON 직렬화 가능한 사전이다.

1. 출입구 옆의 상부 벽 영역과 사용 가능한 상단 영역에서 유효 깊이를 모은다.
2. 고정 seed RANSAC과 SVD로 벽 평면을 추정한다. 바닥 방향 평면이나 낮은 inlier 비율은 거부한다.
3. 출입구 내부 점의 벽 평면에 대한 부호 있는 거리를 비교한다.
4. 벽 뒤 영역 비율이 충분하면 open, 벽 근처 영역이 대부분이면 closed로 판정한다.
   전경 가림·벽 추정 실패·유효 깊이 부족·중간 근거는 unknown이다.

최초 후보 설정은 벽 근처 허용거리 0.08m, 벽 뒤 거리 0.2m,
열림 비율 0.2, 닫힘 비율 0.85, 벽 inlier 비율 0.75다.
전체 설정은 소스 `DEFAULT_CONFIG`와 각 결과의 `prediction.config`에 보존한다.
기존 장면 결과를 보고 임계값을 변경하지 않았다. 채택 기준으로 검증된 값은 아니다.
벽 방향 제한은 카메라가 대체로 수평이라는 가정을 포함하며 기울어진 실물 카메라는 미검증이다.

## 실행과 결과

[진단 실행기](../scripts/door_probe/state_benchmark.py)는 기존 수집 파일의 깊이·보정·기하 정답 ROI를 사용한다.
상태 함수에는 문 배치 각도·정답 상태·world pose·TF·모델 이름을 전달하지 않는다.
정답 ROI 사용 자체가 특권 입력이므로 출력 모드는 `oracle_doorway_roi_development`로 명시한다.

```bash
OPENBLAS_NUM_THREADS=1 /usr/bin/python3 scripts/door_probe/state_benchmark.py \
  --captures experiments/door_probe/run-01 \
  --output experiments/door_probe/state-dev-new
```

실행 결과는 로컬 `experiments/door_probe/state-dev-01/results.json`과 `report.md`에 보존했다.
기존 폴더는 덮어쓰지 않으며 깊이 원본 해시를 확인한다.

| 정답 | 예측 closed | 예측 open | 예측 unknown | 실행 오류 |
|---|---:|---:|---:|---:|
| 닫힘 | 24 | 0 | 3 | 0 |
| 열림 (45°·90°) | 0 | 48 | 6 | 0 |

정확한 상태는 **72/81**, unknown은 **9/81**이다. unknown도 전체 정확도 분모에 포함했다.
문 없는 평벽 27프레임은 출입구 정답이 없어 상태 진단에서 제외했으며,
검출 오검출 성능은 [첫 실험](door-experiment.md)에 별도로 남긴다.

unknown 9개는 세 문 상태의 거리 1m·정면 조건이다. 영상에 주변 벽 영역이 부족해
`insufficient_wall_depth`로 거부됐다. 이 사례를 임의의 열린/닫힌 상태로 채우지 않았다.
조건별 세 RGB가 같고 같은 단순 문 모델을 사용한 개발 자료다.
이 실험은 새로운 독립 장면을 확보한 시험이 아니며 시간에 따른 상태 전이도 평가하지 않았다.

원시 결과 SHA-256:
`ac90474e2ca60774e84a2a69311c91314add9109962e31904b96aa2b1e49067c`.
결과의 `algorithm_sha256`은 실행한 상태 모듈 해시와 일치함을 확인했다.

## 남은 구현과 검증

- 문짝·문틀 라벨과 분할 manifest, 별도 문짝 검출 평가.
- 정답 ROI 대신 RGB-D·검출에서 출입구를 추정하는 경로와 누락을 포함한 전체 평가.
- 같은 문의 시간 종합·상태 전이·오래된 관측 처리.
- LiDAR 지도 좌표의 고정 문 위치·ID와 상태 저장/복원·ROS 표시 연결.
- 새 장면 및 실제 자료 평가. 실물 RGB-D·보정·정답 제공 범위는 🔍 미확인이다.

이 단계는 깊이 후보 모듈과 진단 도구만 구현했다. 전체 후속 목표는 아직 완료되지 않았다.
