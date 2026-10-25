# 하드웨어 구성

> ⚠️ 실물 사양은 연구과제 문서 기준으로 채울 것. 아래 센서 값은 **Gazebo 시뮬레이션(`wf_gazebo`) 설정값**이며 실물과 다를 수 있습니다. (출처: Notion 하드웨어 구성 페이지, `wf_gazebo` revision 미기록 → [setup.md](setup.md#외부-의존성버전-기록))

## 로봇 플랫폼

| 항목 | 내용 |
|---|---|
| 로봇 | JetAuto Ultimate Kit (Hiwonder) |
| 컴퓨터 | Jetson Orin NX 16GB |
| 차체 | 4륜 메카넘 (횡이동 가능) |
| OS / ROS | Ubuntu 22.04 / ROS 2 Humble |

## 센서

| 센서 | 실물 모델·사양 | 시뮬레이션 설정값 | 장착 위치 (시뮬) |
|---|---|---|---|
| 2D LiDAR | 🔍 | 360°, 360빔, 0.15~12 m, 5 Hz | (0.011, 0, 0.164) m, `base_footprint` 기준, 수평 |
| RGB-D 카메라 | 🔍 | 640×480, 15 Hz, 수평 FOV 1.14 rad, 0.2~10 m | 광학 프레임 (0.082373, 0.000079, 0.119864) m, `base_footprint` 기준 |
| IMU | 🔍 | 100 Hz | (0.040, −0.015, 0.078) m, `base_footprint` 기준 |
| 오도메트리 | 엔코더·융합 방식 🔍 | Gazebo `OdometryPublisher`의 `/odom` 20 Hz | 시뮬레이션 자세 기반 |

2026-10-25 외부 `wf_gazebo/urdf/mecanum.xacro` 및 `rosorin.gz.xacro`를 읽어 기준 프레임을 확인했습니다. `base_link`는 `base_footprint`보다 0.028 m 높습니다. 카메라 광학 프레임 높이 0.091864 m는 `base_link` 기준이며, 표의 0.119864 m는 `base_footprint` 기준입니다. `camera_link0`에서 광학 프레임까지 x=0.025 m, rpy=(-π/2, 0, -π/2) 변환이 추가됩니다.

시뮬레이션 차체는 `VelocityControl`, 오도메트리는 `OdometryPublisher`를 사용합니다. 엔코더·휠 미끄러짐·실물 IMU 융합의 검증 결과가 아닙니다. 실물 장착 치수와 보정은 [로봇 인수 절차](robot-acceptance.md)에서 별도로 확인합니다.

## 매핑 관점에서 확인할 것

- [ ] 실물 LiDAR 모델·측정 범위·회전 주기 (5 Hz면 빠르게 움직일 때 지도 번짐 가능)
- [ ] 메카넘 바퀴 미끄러짐으로 인한 오도메트리 오차 — SLAM 입력으로 쓸 때 영향 확인
- [ ] 3D 단계에서 LiDAR를 세로·비스듬히 달 수 있는 브라켓 구성 가능 여부
