# ADR-0003: 연구과제 `wf_gazebo`를 오버레이 워크스페이스로 재사용

- **상태:** ✅ 결정
- **날짜:** 2026-10-21
- **관련 단계:** Phase 0

## 배경

JetAuto 시뮬레이션 모델(URDF·메시), 센서 설정, `ros_gz_bridge` 설정은 연구과제의 `~/worker_following_ws/src/wf_gazebo`에 있다. 이 프로젝트는 그 위에 매핑용 실내 world만 새로 만들면 된다. 다만 연구과제 자원의 공개 범위는 아직 교수님께 확인하지 않았다.

`wf_gazebo/launch/spawn_robot.launch.py`는 `world`, `gui`, 스폰 위치(`x`, `y`, `yaw`)를 인자로 받으므로, world 파일만 바꿔 끼울 수 있다.

## 선택지와 비교

| 기준 | 오버레이 의존 | 이 저장소에 복사 | 별도 저장소로 분리 |
|---|---|---|---|
| 연구과제 자원이 저장소에 들어가는가 | ❌ 들어가지 않음 | ⭕ 들어감 (공개 범위 위험) | ⭕ 들어감 |
| 연구과제 쪽 수정 반영 | 자동 (재빌드만) | 수동 동기화 | 수동 동기화 |
| 이 저장소만으로 재현 | ❌ `worker_following_ws`도 필요 | ⭕ | ❌ |
| 관리 부담 | 낮음 | 중간 (두 벌 관리) | 높음 (저장소·PR 두 벌) |

## 결정과 이유

**오버레이로 의존한다.** `worker_following_ws/install/setup.bash`를 먼저 source한 뒤 이 저장소를 빌드·실행한다. 이 저장소의 `mapping_gazebo`는 world SDF와 `wf_gazebo` launch를 감싸는 launch만 가진다.

- 공개 범위를 확인하기 전에도 저장소에 연구과제 자원이 들어가지 않는다
- 연구과제에서 센서 설정을 고치면(예: LiDAR yaw π 수정, RGB-D 카메라 추가) 그대로 반영된다

## 포기한 것 / 위험

- 이 저장소만 clone해서는 시뮬레이션을 실행할 수 없다. `wf_gazebo`를 쓸 수 있는 사람만 재현 가능하다
- 연구과제 쪽 변경이 이 프로젝트의 평가 결과를 바꿀 수 있다 → 실험 기록에 `wf_gazebo` 상태(수정 날짜 등)를 함께 남긴다 ([setup.md](../setup.md#외부-의존성버전-기록))
- 공개 범위가 "공개 불가"로 정해지면, 공개 URDF로 대체 모델을 만드는 결정이 따로 필요하다

## 검증 방법

2026-10-21 `ros2 launch mapping_gazebo indoor.launch.py gui:=false`로 스폰·센서 토픽·주행을 확인했다. 스폰 위치에서 LiDAR 전·좌·후·우 거리가 도면 기대값과 5 mm 이내였다 ([mapping_gazebo README](../../src/mapping_gazebo/README.md)).
