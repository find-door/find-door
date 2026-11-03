# 검수 bbox의 문짝·출입구 기준별 검출 진단

[ADR-0013](adr/0013-dual-box-evaluation.md)의 오프라인 평가 도구다.
[검수 도구](door-label-review.md)에서 받은 라벨과 기존 검출 원문을 비교하며 모델 추론·학습은 하지 않는다.
도구 구현 당시에는 검수본이 없어 채점 전이었다. 현재는 [사용자 검수 9장 결과](door-reviewed-nine.md)를 별도로 기록했다.

## 입력과 실행

[도구](../scripts/door_probe/label_evaluate.py)의 prepare-view는 완료된 3m 시점 비교 6장 원문을 재사용한다.
추적 요약의 원문 SHA와 각 영상 SHA·크기·방향을 확인하고 현재 manifest의 동일 RGB에 연결한다.
라벨 수정으로 manifest 해시가 바뀌어도 프레임 ID와 원본 RGB 해시가 같으면 검출을 다시 실행할 필요가 없다.

```bash
/usr/bin/python3 scripts/door_probe/label_evaluate.py prepare-view \
  --dataset experiments/door_probe/review-packet-02/dataset.json \
  --output experiments/door_probe/label-eval-new/predictions.json
/usr/bin/python3 scripts/door_probe/label_evaluate.py score \
  --dataset experiments/door_probe/review-packet-02/dataset.json \
  --predictions experiments/door_probe/label-eval-new/predictions.json \
  --environment gazebo --split development \
  --output experiments/door_probe/label-eval-new/report.json
```

검수본을 받으면 score의 --dataset만 새 검수 manifest 경로로 지정한다.
기존 결과는 덮어쓰지 않는다. 모든 선택 프레임의 원본 해시·크기를 확인하며 frame ID·RGB SHA가 다른 예측은 거부한다.
score는 environment(gazebo/real_rgb/real_rgbd)와 split을 각각 하나 요구한다.
train/development/test라는 선언을 실제 독립성의 증명으로 간주하지 않는다.

예측 manifest는 schema_version=1, provenance 객체, frames 목록이다.
각 프레임은 id/rgb_sha256/status/error/detections를 갖는다.
complete는 error=null·검출 목록이며, runtime_error는 설명 문자열·detections=null이다.
검출은 원본 픽셀 xyxy bbox·유한한 [0,1] confidence·door/person class를 갖는다.
person은 공간 진단 수치에서 제외하고 door만 대응한다. 추가 score 문턱은 적용하지 않는다.
입력 provenance와 예측 파일 해시를 결과에 보존하며 도구가 임의 입력의 모델 출처를 인증하지는 않는다.

## 해석과 분모

| 항목 | 정의 |
|---|---|
| leaf / doorway | 같은 원 검출을 각 정답 bbox에 독립적으로 대응한 진단. 합산 금지 |
| draft_frames | 해당 환경·분할에서 검수 전인 프레임. 채점 제외 |
| excluded_visibility_frames | 어느 문이라도 해당 target이 occluded/unknown인 reviewed 프레임. 해당 target 전체 제외 |
| eligible_frames | 해당 target에 정답이 완비된 reviewed 프레임. 문 없는 reviewed 프레임도 포함 |
| frame_exact_success_rate | 모든 GT 대응·추가 FP 없음의 비율. 실행 오류·예측 누락도 분모에 포함 |
| valid_inference_precision / recall / f1 | 유효 추론에 한정한 조건부 지표. 전체 성능으로 해석 금지 |
| matched_mean_iou | 대응된 TP만의 조건부 평균 IoU. AP/mAP 또는 모든 GT의 위치 정확도가 아님 |
| eligible_gt_objects / valid_inference_gt_objects | 전체 유효 정답 수 / 실제 유효 추론에서 채점한 정답 수 |
| selected_prediction_frames / selected_frames_without_prediction | 선택 자료 중 기존 예측이 있는 수 / 아직 없는 수. draft도 이 입력 범위 집계에 포함 |

일대일 대응은 IoU≥0.5의 최대 개수를 찾는다. IoU가 가장 큰 쌍부터 확정하는 단순 greedy 방식은
두 문이 겹칠 때 TP를 놓칠 수 있어 증강 경로로 다시 연결한다.
IoU 합이 최대라는 보장은 없으며 같은 입력에 결정적인 대응을 제공한다.
P/R은 각각 분모가 없으면 null이다. F1은 2TP/(2TP+FP+FN_valid)로 직접 계산하므로 양성 무검출·음성 FP에서는 0이며 모든 count가 0일 때 null이다.
분모가 없는 지표는 null로 기록한다. 초안을 무검출 정답·0% 성능으로 바꾸지 않는다.

상태는 awaiting_review, no_selected_frames, no_evaluable_targets, incomplete_predictions,
evaluated_with_runtime_errors, evaluated로 구분한다.
evaluated는 지표 계산이 끝났다는 뜻이며 모델이 과목 채택 기준을 통과했다는 뜻은 아니다.
두 target의 가시성 분모가 다르면 점수 차이를 그대로 모델 개선으로 해석하지 않는다.
출입구 추정 모듈·깊이 상태·문 지도는 이 원 검출 bbox 진단에서 평가하지 않는다.

## 도구 구현 시점의 검수 전 준비 상태

2026-11-03 `label-eval-03`에서 기존 6장 원문으로 예측 manifest를 준비하고 세 환경의 준비 상태를 확인했다.
모델 추론·학습을 실행하지 않았다. [추적 요약](data/door-label-evaluation-summary.json)에 입력·출력·평가 도구 해시를 남겼다.

| 환경·분할 | 선택 프레임 | reviewed | 이번 파일의 예측 | 상태 |
|---|---:|---:|---:|---|
| Gazebo / development | 45 | 0 | 6 | awaiting_review |
| 실제 RGB / unassigned | 50 | 0 | 0 | awaiting_review |
| 실제 RGB-D / unassigned | 0 | 0 | 0 | no_selected_frames |

문짝·출입구 모두 유효 채점 프레임은 0이고 P/R/F1·프레임 exact 성공률·평균 IoU는 null이다.
이는 모델 성능이 0이라는 뜻이 아니라 당시 정답 검수 전이었다는 뜻이다.
Gazebo 39장과 실제 RGB 50장의 예측 미연결을 무검출로 바꾸지 않았다.
실제 RGB의 과거 C 원문은 [별도 참고 감사](door-dataset.md#기존-실제-사진-c-결과의-참고-감사)가 있으며,
이번 prepare-view 파일은 그 결과를 자동 수입하지 않는다.
앞 9장을 검수하더라도 이 파일에는 3m 6장만 있어 2m 3장의 정확한 RGB 해시에 대응하는 예측은 추가로 준비해야 한다.
누락 상태의 보고서를 전체 평가셋 성능으로 채택하지 않는다.

검수 경계·대응·음성/무검출·오류·누락·환경 분리·해시·원본 크기·기존 예측 재사용·덮어쓰기 거부 등 32검사를 통과했다.
전체 오프라인 769개 통과·ROS 전용 5개 건너뜀이다. ROS 제품 소스는 바꾸지 않아 패키지 재빌드는 하지 않았다.
이 표는 원문을 재사용한 초기 준비 상태 검사다. 이후 [9장 검수·채점](door-reviewed-nine.md)을 완료했다.
독립 평가·학습 필요성의 최종 결론은 여전히 남아 있다.
