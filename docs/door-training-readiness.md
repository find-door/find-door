# YOLO11 학습 후보 준비 상태

2026-11-03 [사용자 검수 9장 기준선](door-reviewed-nine.md)의 검출 누락 이후 학습 자원과 자료를 점검했다.
이 문서는 학습 후보 계획이며 학습 실행·모델 채택·검출 개선 완료를 의미하지 않는다.
[점검 원문과 입력·소스 해시](data/door-training-readiness-summary.json)를 보존했다.

## 확인한 사실

아래 표는 최초 점검 시점이다. 이후 [실제 AI 라벨](door-ai-labels.md)을 작성하고
[출처 그룹별 개발 학습 입력](door-training-data.md)을 준비했다. 사람 검수·독립 시험 준비와 구분한다.

| 항목 | 확인 결과 |
|---|---|
| 검수된 자료 | 동일 Gazebo family/development 그룹의 양성 9장 |
| 검수된 음성 | 0장 |
| 실제 RGB 이중 bbox 검수 | 0장. 원본 사진 50장은 있으나 새 두 bbox는 draft |
| 실제 RGB-D | 사용자 현재 자료 없음 확인 |
| 기존 venv | ultralytics 8.4.191, torch 2.14.1+cu126, torchvision 0.29.1+cu126 |
| YOLO11 설정·학습 구현 | 로컬 yolo11.yaml과 detection trainer 파일 존재 |
| 확인한 모델 폴더 | recon-report/models에 yoloe-26n-seg.pt만 확인. YOLO11 pretrained 파일 미확인 |
| GPU | RTX 3050 Laptop 4GB, 총 4096MiB. 점검 시 free 3683MiB |

로컬 COCO 설정은 80클래스이며 door가 없다. YOLO11 기본 pretrained 모델을 그대로 교체해 문 검출기로 쓸 수는 없다.
공식 문서도 COCO 80클래스 사전 학습과 사용자 데이터 학습 경로를 구분한다.
[YOLO11 공식 안내](https://docs.ultralytics.com/models/yolo11/), [학습 공식 안내](https://docs.ultralytics.com/modes/train/).
로컬 구현 파일 존재는 실제 학습 실행·CUDA 학습 호환·최대 메모리 사용량 검증과 다르다.

## 다음 학습 후보와 선행 조건

소형 후보는 사용자 제안 범위의 YOLO11n이다. 공식 release `yolo11n.pt` asset 크기는
5,613,764bytes(약 5.61MB/5.35MiB)로 API metadata에서 확인했다.
[공식 release metadata](https://api.github.com/repos/ultralytics/assets/releases/tags/v8.4.0),
[가중치 asset](https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo11n.pt).
이번 점검에서 다운로드하지 않았으므로 checkpoint SHA는 아직 없다. 설치·다운로드 대상은 실행 전에 범위를 확정한다.

현재 9장만 프레임 단위로 무작위 분할하면 같은 문·같은 장면이 학습과 시험에 들어간다.
증강한 사본도 같은 원본 그룹이다. 이를 독립 시험으로 부르거나 학습 개선율을 제출 결과로 사용하지 않는다.
다음 순서로 준비한다.

1. 다양한 문·관측각·문 없는 장면의 라벨과 문/장소/세션 그룹을 확인한다.
2. 학습/검증/시험을 그룹 단위로 나누고 각 역할을 고정한다. 현재 development 9장은 개발 진단으로 보존한다.
3. 문짝과 출입구 중 학습 target을 명시한다. 서로 다른 두 bbox를 같은 라벨 의미로 섞지 않는다.
4. 현재 YOLOE와 학습 후보에 동일 시험 RGB·정답·IoU 기준을 적용하고 오류·누락을 함께 기록한다.
5. 데이터·checkpoint·설정·학습 seed·실행 환경·자원·결과 해시를 보존한다.

현재 점수는 누락이 주된 문제라는 개발 근거지만 어떤 학습 target·모델이 실제로 개선할지는 🔍 미검증이다.
실제 RGB 50장을 검수하면 Gazebo와 분리한 검출 평가 자료를 마련할 수 있지만,
장소·세션 그룹 확인 전에는 그 사진들을 임의로 학습·시험에 섞지 않는다.
RGB 검수로 깊이 상태나 미터 단위 지도 위치를 실제 환경에서 검증했다고 표현하지 않는다.

## 사용자 검수 이어가기

기존 `review-packet-02/review.html`을 열고 제출한 `door-label-patch.json`을 **수정본 이어서 열기**로 불러온다.
앞 9장 작업을 유지한 채 실제 RGB 사진은 화면 **46~95번**에서 이어서 검수할 수 있다.
사용자가 실제 RGB 50장도 이어서 검수 가능하다고 응답했다. 아직 새 수정본을 받지 않았으므로 검수 완료는 기존 9장으로 유지한다.

프로그램·센서·기존 체크포인트·외부 워크스페이스는 변경하지 않았다.
실행한 것은 파일·버전·GPU 메타데이터·공식 문서·공식 asset metadata의 읽기 전용 점검이다.
학습·다운로드·설치·새 추론·Gazebo 실행은 수행하지 않았다.
