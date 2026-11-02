# ADR-0009: 문 위치·상태 ROS 출력과 시각별 TF 연결

- **상태:** ✅ 결정 — 개발 후보 연결, 알고리즘 일반 성능 채택과 구분
- **날짜:** 2026-11-03
- **관련 단계:** Find Door 후속 구현

## 배경

문짝 표면을 일반 정적 객체로 저장하면 열린 문에서 문 위치가 이동할 수 있다.
출입구 추정·벽 깊이 판단·문 메모리 후보를 관측 시각 TF와 연결하고 상태 속성을 별도로 보존해야 한다.

## 선택지와 비교

| 선택지 | 장점 | 제약 |
|---|---|---|
| 기존 정적 객체 class에 door_open/door_closed 추가 | 기존 표시 재사용 | 상태마다 객체가 나뉘고 문짝 표면을 고정 문턱으로 오인할 수 있음 |
| 문 전용 ROS 노드·개발 JSON 출력 | 위치·상태 분리와 검증 범위 명시 | 외부 검출 입력과 별도 RViz topic 설정 필요 |
| 전용 메시지 패키지 | 필드 타입을 ROS에서 강제 | 현재 개발 연결보다 패키지 범위가 커짐 |

## 결정과 이유

`door_mapper`는 RGB/depth/CameraInfo/Detection2DArray의 exact 시각과 광학 프레임을 확인한다.
출입구 추정·깊이 상태·문턱 후보를 구하고 관측 시각 map←camera TF로 옮긴다.
TF가 없으면 대기 후 거부하며 최신 TF로 대체하지 않는다.
동일 문의 ID·고정 위치·시간 상태는 DoorMap으로 관리한다.

출력은 `/semantic/door_map`의 `std_msgs/msg/String` JSON과
`/semantic/door_markers`의 `visualization_msgs/msg/MarkerArray`로 제공한다.
JSON은 개발 계약이며 map_id/frame/현재 시각/마지막 관측 시각/epoch/객체/카운터를 포함한다.
마커와 지도 출력은 TRANSIENT_LOCAL이다. 노드는 TF나 주행 명령을 발행하지 않는다.

저장·복원은 `/semantic/doors/save`, `/semantic/doors/load`의 `std_srvs/srv/Trigger`로 명시적으로 수행한다.
`door_session_schema_version=1` 문서는 보정 기준과 DoorMap 문서를 포함한다.
기존 정적 객체 schema와 분리하며, 파일 덮어쓰기를 거부하고 현재 map_id·frame·설정·지도 보정과 맞는 파일만 복원한다.

큰 map→odom 보정은 기존 정책과 같이 0.2m 또는 yaw 0.1rad 초과 시 초기화한다.
시간 역행은 메모리·대기 관측·TF 버퍼·동기화 큐를 비워 이전 epoch TF를 재사용하지 않게 한다.
표시 TTL은 현재 ROS 시각으로 판단하지만 대기 관측의 메모리 입력 watermark를 앞당기지 않는다.

## 포기한 것 / 위험

검출은 최고 score 문 하나를 사용하며 복수 문 동시 관측은 아직 처리하지 않는다.
출입구 후보는 보이는 돌출 문틀과 균일 벽 등 제한된 가정에 의존한다.
일반 실내 성능·실제 영상·지도 정확도는 이 연결 결정으로 보장하지 않는다.
실시간 검출기가 별도로 원본 시각 Detection2DArray를 발행해야 한다.
입력 허용 지연은 기본 10초지만 상태 TTL은 기본 2초이므로 오래 걸린 검출은 위치가 처리돼도 상태가 unknown일 수 있다.

## 검증 방법

[ROS 재생 진단](../door-ros.md)에서 실제 저장 RGB-D·C 검출을 ROS 메시지로 발행하고
상태 전환·ID·마커·저장/복원·입력 거부·TTL·지도 보정·시각 역행과 TF epoch 격리를 확인한다.
항등 map→odom을 제공하는 인위적 재생과 실제 LiDAR SLAM 검증을 구분한다.
