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
현재 생성본은 86프레임이며 모든 검수 완료 체크를 해제한 상태로 시작한다.

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

## 한계와 다음 검수

이 도구로 만들어진 파일을 바로 독립 시험셋이나 학습 준비 완료 자료라고 부르지 않는다.
특히 프레임에 문이 빠졌는지, bbox 의미가 일관적인지, 실제 장면이 같은 그룹인지 사람이 확인해야 한다.
실제 자료는 현재 collection별 unassigned다. 장면·촬영 세션 검수 후 별도로 분할한다.
RGB 사진에 깊이·CameraInfo·TF를 추가하거나 실제 RGB-D 평가를 대체하지 않는다.
현재 사람이 검수한 새 이중 bbox는 0프레임이며 학습은 아직 수행하지 않았다.

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
