# ADR-0002: 개발 환경으로 Ubuntu 22.04 + ROS 2 Humble + Gazebo Harmonic 사용

- **상태:** ✅ 결정
- **날짜:** 2026-10-21
- **관련 단계:** Phase 0

## 배경

개발 PC(시뮬레이션)와 로봇(JetAuto)에서 같은 OS·ROS 버전을 써야 코드·launch·파라미터를 그대로 옮길 수 있다. 시뮬레이터는 연구과제 모델(`wf_gazebo`)과 맞아야 한다.

## 선택지와 비교

| 기준 | Humble + Fortress | Humble + Harmonic | Jazzy + Harmonic |
|---|---|---|---|
| OS | Ubuntu 22.04 | Ubuntu 22.04 | Ubuntu 24.04 |
| ROS 공식 조합 | ✅ 공식 기본 | ❌ 비기본 (OSRF 저장소의 `ros-humble-ros-gzharmonic`) | ✅ 공식 기본 |
| JetAuto(로봇) 환경과 일치 | ✅ | ✅ | ❌ (로봇은 22.04 / Humble) |
| 연구과제 시뮬레이션(`wf_gazebo`) 재사용 | ❌ (Harmonic 기준이라 이식 필요) | ✅ 그대로 사용 | △ Gazebo 쪽은 동일, ROS 쪽(launch·패키지)은 Jazzy 이식 필요 |
| ROS 지원 기간 | 2027-05까지 | 2027-05까지 | 2029-05까지 |

## 결정과 이유

**Ubuntu 22.04 + ROS 2 Humble + Gazebo Harmonic**을 사용한다.

- 로봇(Jetson Orin NX)이 Ubuntu 22.04 / ROS 2 Humble이므로 개발 PC도 같은 버전으로 맞춘다
- 연구과제 시뮬레이션(`wf_gazebo`)이 이미 Gazebo Harmonic을 사용 중이므로, 같은 Harmonic을 써서 모델·센서 설정을 이식 없이 재사용한다

## 포기한 것 / 위험

- Humble의 공식 Gazebo는 Fortress다. `ros-humble-ros-gzharmonic`은 비공식 바이너리라 Humble 쪽 변경 후 재빌드가 늦어질 수 있다
- `ros-humble-ros-gz*`(Fortress용) 패키지, 그리고 Gazebo에 의존하는 공식 ROS 우분투 패키지와 함께 설치할 수 없다. 필요한 패키지가 Fortress용 `ros_gz`에 의존하면 소스 빌드가 필요하다
- Gazebo Classic에 의존하는 패키지와 충돌할 수 있다

## 검증 방법

Phase 0 완료 조건: 이 환경에서 실내 world를 띄우고 JetAuto를 시뮬레이션 주행한다. 설치 절차는 [setup.md](../setup.md)에 기록하고 실제 실행 날짜를 남긴다.
