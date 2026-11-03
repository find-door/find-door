# 문짝·출입구 라벨 작성·검수

[자료 계약](door-dataset.md)의 초안을 이미지 위에서 수정하는 로컬 도구다.
원본 이미지·기존 라벨은 읽기만 하고 별도 HTML·수정본 JSON·새 manifest를 생성한다.
모델 추론·자동 정답 생성·학습은 수행하지 않는다.

## 실행

저장소 루트에서 실행한다. Python Pillow와 기존 자료 계약 모듈을 사용하며 ROS는 필요 없다.

```bash
/usr/bin/python3 scripts/door_probe/label_review.py export \
  --manifest experiments/door_probe/dataset-intake-03/dataset.json \
  --output experiments/door_probe/dataset-intake-03/review.html
```

생성한 HTML을 로컬 브라우저로 연다. 사진을 포함한 단일 파일이며 서버·외부 네트워크 요청이 없다.
원본 자료를 포함하므로 HTML도 원본과 같은 공개 범위로 취급하고 Git에 추가하지 않는다.
최초 생성본은 86프레임이며 모든 검수 완료 체크를 해제한 상태로 시작한다.

1. 프레임을 선택하고 문 ID와 문짝/출입구 대상을 선택한다.
2. 이미지 위에서 드래그해 보이는 영역의 bbox를 그린다. 문짝은 파랑, 출입구는 주황이다.
3. 완전 가림·불명은 해당 버튼으로 bbox를 null로 바꾼다. 일부 가림은 보이는 박스를 사용한다.
4. 아래 JSON에서 문 ID·state를 수정하고 **JSON 적용**을 누른다. 반열림도 open이다.
   문 추가·삭제로 누락과 오검출을 수정하고, 문이 없는 프레임은 빈 목록으로 만든다.
5. 모든 문·상태·두 bbox·누락 여부를 확인한 프레임만 검수 완료를 체크한다.
6. **수정본 JSON 저장**으로 내려받는다. 페이지를 닫으면 저장하지 않은 작업은 사라진다.
   다시 열 때 **수정본 이어서 열기**로 같은 원본의 수정본을 불러온다.
   불러오기는 현재 편집을 대체하므로 필요한 작업은 먼저 JSON으로 저장한다.

라벨을 수정하면 해당 프레임의 검수 완료 표시가 해제된다.
새로 export할 때 기존 reviewed 표시는 자동 승계하지 않는다.
JSON을 통한 추가 필드는 저장되지만 필수 라벨·가시성·bbox는 계약 검증을 거친다.
기존 단일 bbox를 새 두 bbox로 자동 복제하지 않는다.

```bash
/usr/bin/python3 scripts/door_probe/label_review.py apply \
  --manifest experiments/door_probe/dataset-intake-03/dataset.json \
  --patch /경로/door-label-patch.json \
  --reviewer 검수자이름 \
  --output experiments/door_probe/dataset-intake-03/dataset-reviewed-01.json
```

검수 완료 체크가 하나라도 있으면 검수자 이름이 필수다. 미체크 프레임은 draft로 저장한다.
검수자·UTC 적용 시각, 원본 manifest·수정본·적용 도구 해시를 결과에 남긴다.
이는 사용자가 선언한 검수 기록이며 실제 사람의 신원·정답 품질을 도구가 증명하지 않는다.
원본 manifest·영상 해시·크기·EXIF 방향과 전체 프레임 대응을 확인하고,
수정본의 split·group·원본 경로 변경과 프레임 누락·중복을 거부한다.
원본의 group/split/capabilities/legacy_labels는 보존하며 파일은 덮어쓰지 않는다.
결과를 다른 폴더에 저장해도 영상을 찾도록 기존 상대 RGB 경로는 절대 경로로 고정한다.

## 실패·보조 시점을 포함한 95장 검수 묶음

`v0.2.0-door-review.1`에서 [검수 묶음 생성기](../scripts/door_probe/review_packet.py)로
기존 86장에 실제 Gazebo RGB 9장을 추가했다. 새 영상은 먼저 보여주며 원본 파일을 복사하거나 덮어쓰지 않는다.
현재 로컬 화면은 `experiments/door_probe/review-packet-02/review.html`이다.
먼저 앞의 9장에서 **문짝 박스와 출입구 박스를 각각 작성하고 검수 완료를 체크**한다.
문짝은 보이는 부분, 출입구는 문틀을 포함한 영역이다. 일부가 영상 밖이면 영상 안의 영역을 기록한다.
문짝이 완전히 가렸거나 판단 불가하면 해당 가시성 버튼으로 bbox를 null로 둔다.
완료한 만큼 **수정본 JSON 저장**을 눌러 보관한다. 전체 95장을 한 번에 검수할 필요는 없다.

| 앞쪽 순서 | 영상 | 새 프레임 수 |
|---|---|---:|
| 1~3 | 평판 금속문 2m 닫힘·반열림·완전 열림 | 3 |
| 4~9 | 같은 금속문 3m의 낮은/높은 시점 세 쌍 | 6 |
| 10~95 | 기존 Gazebo 36장과 실제 RGB 50장 | 86 |

**촬영·검출 참고**는 읽기 전용이다. 예측 bbox·점수는 정답으로 자동 복제하지 않는다.
새 9장 모두 두 bbox는 null, 검수 상태는 draft로 시작한다. 배치 상태는 촬영 정답으로 기록한다.
캐시 재관측의 성공 영상을 새 문 검출 성공으로 바꿔 넣지 않았으며, 최초 검출 실패 조건도 그대로 포함했다.

```bash
/usr/bin/python3 scripts/door_probe/review_packet.py \
  --base experiments/door_probe/dataset-intake-03/dataset.json \
  --output experiments/door_probe/review-packet-new
```

묶음 생성기는 기존 NumPy 기반 기하 검증도 사용하며 ROS 실행은 필요 없다.
새 manifest를 만들 때 기존 라벨·검수 기록은 보존한다. 같은 파생 Gazebo 장면은 보수적으로
`gazebo_door_probe_v1_family`의 development 그룹에 묶고 실제 RGB 50장은 unassigned로 유지한다.
새 HTML을 export할 때는 기존 정책에 따라 검수 체크를 모두 해제한다. 라벨 박스 보존과 검수 체크 승계는 별개다.
이미 86장 화면에서 저장한 수정본은 95장 manifest에 바로 적용할 수 없다.
먼저 그 수정본을 원래 manifest에 적용한 뒤 위 생성기의 --base로 결과를 지정한다.
같은 RGB가 다른 분할·환경·상태에 있는 경우에는 자동으로 합치지 않고 거부한다.

새 9장은 깊이 배열을 저장하지 않은 RGB이므로 depth=false다. 3m 여섯 장은 CameraInfo 기록이 있고,
낮은 시점만 관측 시각 TF 기록이 있다. 높은 시점의 SDF 고정 pose를 TF 기록으로 표시하지 않는다.
원문·영상 해시·크기·방향·촬영 보정 조건을 검사한다.
95장 묶음 생성 당시 이중 bbox 검수 완료는 0장이었다. 이후 제출받은 [9장 검수본 적용·평가](door-reviewed-nine.md)를 완료했다.
기존 화면에서 계속 검수하려면 제출했던 JSON을 **수정본 이어서 열기**로 불러온다. 앞의 9장을 다시 그릴 필요는 없다.

```bash
/usr/bin/python3 scripts/door_probe/label_review.py apply \
  --manifest experiments/door_probe/review-packet-02/dataset.json \
  --patch /경로/door-label-patch.json --reviewer 검수자이름 \
  --output experiments/door_probe/review-packet-02/dataset-reviewed-new.json
```

95장 Firefox headless 검사는 이미지·읽기 전용 참고·박스 드래그·검수 체크·로딩/복원 잠금·수정본 객체 생성까지 통과했다.
검사에서 임시 편집한 박스를 검수 정답으로 저장하지 않았다. 실제 다운로드 위치 선택은 자동 검사 범위 밖이다.
묶음·수정본 관련 20개 검사와 전체 오프라인 737개 검사가 통과했고 ROS 전용 5개는 건너뛰었다.
ROS 제품 소스를 변경하지 않아 이번 단계에서 패키지를 재빌드하지 않았다.
[추적 요약](data/door-review-packet-summary.json)에 데이터·HTML·도구·부모 실험의 해시를 남겼다.

검수본을 적용한 뒤 [검수 bbox 평가 도구](door-label-evaluation.md)로 기존 검출 원문과 비교할 수 있다.
검수하지 않은 프레임은 채점하지 않으며 같은 원본 RGB의 예측을 재사용하므로 라벨 수정만으로 재추론하지 않는다.

## 한계와 다음 검수

이 도구로 만들어진 파일을 바로 독립 시험셋이나 학습 준비 완료 자료라고 부르지 않는다.
특히 프레임에 문이 빠졌는지, bbox 의미가 일관적인지, 실제 장면이 같은 그룹인지 사람이 확인해야 한다.
실제 자료는 현재 collection별 unassigned다. 장면·촬영 세션 검수 후 별도로 분할한다.
RGB 사진에 깊이·CameraInfo·TF를 추가하거나 실제 RGB-D 평가를 대체하지 않는다.
현재 사용자 검수본을 적용한 새 이중 bbox는 9프레임이고 나머지 86프레임은 draft다. 학습은 아직 수행하지 않았다.
이후 사용자 요청으로 [실제 사진 50장의 AI 시각 라벨](door-ai-labels.md)을 별도로 작성했다.
사용자 검수 9장과 AI 검토 41장·보류 9장의 출처를 구분하며, AI 라벨을 사람 정답으로 자동 승격하지 않는다.

## 실행 검증 기록 — 2026-11-03

`v0.2.0-door-labels.1` 단계에서 Python 수정본 검사 12개와 기존 자료 계약 검사 39개가 통과했다.
전체 오프라인 검사도 673개 통과, ROS 전용 검사 2개 건너뜀으로 마쳤다.
기존 86프레임을 HTML로 생성하고, 수정 없는 별도 patch를 적용해 86 draft / 0 reviewed를 확인했다.
Firefox 158 headless에서 실제 pointer 입력으로 박스를 그리고 프레임 이동·검수 체크·잘못된 JSON 중
체크 해제·이미지 로딩 및 수정본 복원 중 편집 차단·복원·patch 객체 생성을 검사했다.
재현 명령은 다음과 같으며 GUI 창을 띄우지 않는다. Node.js 24의 WebSocket과 Firefox BiDi를 사용한다.
검사 도구는 시험 중 편집한 박스를 사람 검수 manifest로 저장하지 않는다.
브라우저 다운로드 창·디스크 저장 경로 선택은 자동 검증하지 않았으며 생성 patch 객체를 확인했다.

```bash
node scripts/door_probe/review_browser_check.mjs \
  experiments/door_probe/dataset-intake-03/review-03.html \
  experiments/door_probe/dataset-intake-03/review-browser-check-new.json
```

[추적 요약](data/door-label-review-summary.json)에 원본·도구·HTML·결과 해시를 기록했다.
HTML 생성 시 특수 문자를 포함한 ID의 왕복 보존을 검사했다.
원본 이미지를 포함한 생성 HTML과 시험 원문은 Git 제외 로컬 경로에 보존한다.
