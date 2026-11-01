# 로드맵

단계별로 쌓아가며, 각 단계가 끝날 때마다 동작 영상과 수치 결과를 남깁니다.

> 🎯 **완성 기준 (MVP): Phase 1 + Phase 2.**
> 실물 JetAuto로 실제 공간의 2D 지도를 만들고, 그 위에 카메라로 인식한 객체를 표시한 데모 영상 + 정량 결과가 있으면 1차 완성으로 봅니다.
> Phase 3~4는 확장 목표로, 1차 완성 이후 진행합니다.

| 단계 | 목표 시점 | 완료 조건 | 릴리스 태그 (예정) |
|---|---|---|---|
| Phase 0 개발 환경 | 🔍 | 실내 world에서 JetAuto 시뮬레이션 주행 | `v0.1.0` |
| Phase 1 2D SLAM | 🔍 | 실물로 만든 지도의 도면 대비 오차 측정 완료 | `v0.2.0` |
| Phase 2 카메라 융합 | 🔍 | 지도 위 객체 표시 + 위치 정확도 측정, 데모 영상 | `v1.0.0` (MVP) |
| Phase 3 3D (확장) | 🔍 | 3D 또는 층 구분 지도 생성 | `v1.1.0` |
| Phase 4 드론 (확장) | 🔍 | 기울기 보정 매핑 검증 | `v1.2.0` |

## Phase 0 — 개발 환경

- [x] 개발 환경 결정 — Ubuntu 22.04 + ROS 2 Humble + Gazebo Harmonic ([ADR-0002](adr/0002-dev-environment.md))
- [x] 개발 PC에 환경 설치 후 [setup.md](setup.md) 확인 날짜 기록 (2026-10-21)
- [x] 작업자추종로봇 과제의 JetAuto Gazebo 모델(`wf_gazebo`) 재사용 범위 확인 — 오버레이로 의존 ([ADR-0003](adr/0003-wf-gazebo-reuse.md))
- [x] 매핑용 실내 world 만들기 (복도·방·문이 있는 구조, 치수를 아는 도면 기준) — `mapping_gazebo/worlds/indoor_v1.sdf`
- [x] 로컬 저장소 생성, 문서 뼈대·기여 규칙 작성
- [x] GitHub Private 원격 저장소 생성 — 공개 전환 승인과 분리
- [x] ROS 2 워크스페이스·패키지 뼈대 생성 — `mapping_gazebo`
- [ ] 실내 world에서 시뮬레이션 주행 확인 — 자동 주행·실제 Gazebo 정지 장면 캡처 완료, RViz2 수동 화면 확인 남음
- [ ] 연구과제 자원(JetAuto, `wf_gazebo`) 사용 및 GitHub 공개 범위를 교수님께 확인

## Phase 1 — 2D SLAM (지상 로봇)

- [x] slam_toolbox 초기 기준선 launch·파라미터 및 [인터페이스 계약](interfaces.md) 작성
- [x] 입력 수신·프레임·시간·TF·지도 메시지 검사 도구 구현
- 정확도 평가는 아래 실험에서 별도 수행하며, 기준선 구현을 알고리즘 최종 선정으로 간주하지 않습니다.

- [x] 시뮬 SLAM 개발 기준선 비교·결정 — [ADR-0004](adr/0004-prehardware-slam-baseline.md)
- [ ] 실물 센서·독립 정답·Jetson 자원으로 최종 SLAM 선정
  - Hector는 upstream이 ROS 1 기반이고 루프 클로저가 없음 → ROS 2 포트 존재 여부 확인 후 조건부 후보로 비교
- [x] 시뮬레이션에서 2D 지도 생성 → 도면 대비 오차 측정 — [3회 자동 주행 평가](evaluation.md#자동-주행지도-기하-평가--2026-10-25)
- [x] 루프 클로저 ON/OFF 각 3회 비교 — [급변 재현 안 됨](loop-closure-investigation.md#2026-10-25-결과), 원인 확정 아님
- [x] 실패 bag 출력 TF 제거·동일 기록 입력 재생 — [재생 결과](replay-evaluation.md)
- [x] SLAM 시작 전 입력 기록·TF/그래프 진단·오프라인 coverage 검사 — [실행과 한계](slam-startup-capture.md)
- [x] 새 기록(v2) 급변 원인 확정 — 스캔 무게중심 기반 루프 후보가 벽 너머 방을 연결 ([원인 분석](loop-closure-investigation.md#2026-10-26-원인-분석-스캔-무게중심-기반-루프-후보-선택))
- [ ] 과거 예비 실패(3.566 m)와 같은 원인인지 확인 — 초기 scan 미기록으로 🔍
- [x] `use_scan_barycenter: false` 기본값 채택 — 온라인 5회 통과 ([ADR-0004](adr/0004-prehardware-slam-baseline.md#2026-10-27-루프-후보-기준-변경))
- [x] 구조가 다른 `indoor_v2`에서 재검증 — [현재 기본값 5회 모두 실패](loop-closure-investigation.md#2026-10-31-indoor_v2-재검증)
- [x] `indoor_v2` 큰 지도 실패 원인 확인 — [재생한 실패 bag 2개에서 수락된 벽 너머 오연결](loop-closure-investigation.md#루프-수락-기록과-정답-비교)
- [x] 처리 간격 스윕 — [0.1–0.5 m 중 사전 기준 통과 없음](loop-closure-investigation.md#처리-간격-스윕과-조사-결론), 시뮬레이션 튜닝 종료
- [ ] 실물 odom 오차 측정 후 루프 설정 재평가 🔍
- [ ] `indoor_v2` 위치 오차가 스캔 매칭 단계에서 한쪽으로 쌓이는 이유 확인 🔍
- [x] 실물 배포용 런타임·프로파일·센서 점검·기록/격리 재생·자원 기록 준비 — [검증](robot-readiness-evaluation.md), ARM64·실물은 미검증
- [ ] 실물 JetAuto로 실제 공간(복도·실험실) 매핑, 줄자 실측과 비교
- [ ] 루프 클로저 전·후 비교 (루프 클로저가 없는 후보는 N/A)

## Phase 2 — 카메라 융합 (시맨틱 지도)

- [x] 별도 검출·RGB-D 투영 구조 결정 — [ADR-0005](adr/0005-rgbd-projection.md)
- [x] 실제 Gazebo 색상 표적 투영·이동·ID·저장·복원 검증 — [결과](semantic-evaluation.md)
- [x] NanoDet CPU/ROS와 외부 검출 연결 — [ADR-0007](adr/0007-cpu-detector-baseline.md), 실제 목표 클래스 성능 미검증
- [x] 정적 객체·동적 관측 분리·지도 보정 정책 — [ADR-0008](adr/0008-semantic-map-policy.md)

- [x] 16UC1(mm)/32FC1(m)·RGB/BGR·선택적 bounded sync 연결 검증 — 물리 정렬 보정과 구분
- [ ] 카메라↔LiDAR 외부 보정 (장착 위치·TF)
- [ ] 객체 인식 (문·계단·사람·소화기 등) → 지도 좌표로 투영
- [ ] 지도 위 표시 위치 정확도 측정

## Phase 3 — 3D 확장

- [ ] 방식 비교: 세로 장착 / 비스듬히 장착 / 회전 LiDAR / depth 카메라
- [ ] 3D 포인트클라우드 또는 층 구분 지도 생성

## Phase 4 — 드론 적용 검토

- [ ] 기울기(roll/pitch) 보정 — IMU 또는 짐벌
- [ ] 바닥·천장 점 제거, 높이 변화 대응
- [ ] 휠 오도메트리 없는 운동 추정 (VIO 또는 LiDAR 기반)
- [ ] 드론 시뮬레이션 또는 핸드헬드 리그로 드론 움직임 흉내내기
- [ ] (확장) 연기 환경 — 열화상 카메라 융합 검토, 여러 대 지도 병합

## 로봇 도착 전 소프트웨어 준비

[작업 범위와 상태](prehardware-plan.md)에 실행·재생·시맨틱 합성 검증·CI를 정리합니다.
[실물 인수 절차](robot-acceptance.md)는 하드웨어 도착 후 별도로 수행합니다.
