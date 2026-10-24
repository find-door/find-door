# mapping_slam

slam_toolbox를 이용하는 초기 2D SLAM 기준선입니다. 최종 알고리즘 선정과 정확도 평가는 ADR-0004에서
진행하며, 현재 파라미터는 indoor_v1의 연결 검증용입니다.
센서·TF 계약은 [interfaces.md](../../docs/interfaces.md)를 따릅니다.

## 준비

```bash
sudo apt-get install ros-humble-slam-toolbox
source /opt/ros/humble/setup.bash
source ~/worker_following_ws/install/setup.bash
# 현재 checkout/worktree 루트에서 실행
colcon build --symlink-install
source install/setup.bash
```

## 실행

```bash
# 에이전트 검증용 격리. 모든 관련 터미널에 동일하게 적용
export ROS_DOMAIN_ID=12 GZ_PARTITION=codex
ros2 launch mapping_slam simulation.launch.py
```

통합 launch 인자: `world`, `gui`(기본 false), `headless_rendering`, `x`, `y`, `yaw`, `params_file`.
world·스폰 기본값은 [mapping_gazebo](../mapping_gazebo/README.md)와 같습니다.

시뮬레이션이 이미 실행 중이라면 아래 단독 launch만 사용합니다.

```bash
ros2 launch mapping_slam mapping.launch.py use_sim_time:=true
# 다른 설정 파일 사용
ros2 launch mapping_slam mapping.launch.py use_sim_time:=true params_file:=/absolute/path/slam.yaml
```

단독 launch의 `use_sim_time` 기본값은 false입니다. 실물 드라이버 연결은 아직 검증하지 않았습니다.
프레임·토픽 변경은 `params_file`로 전달하고 센서 계약도 함께 확인합니다.

## 입력·지도 검사

환경을 동일하게 준비한 별도 터미널에서 실행합니다.

```bash
ros2 run mapping_eval check_interfaces --timeout 30 --require-map
```

수동 주행은 `teleop_twist_keyboard`를 사용합니다. 자동 launch는 주행 명령을 보내지 않습니다.

## 지도 저장과 재로딩

OccupancyGrid 지도(PGM + YAML) 저장 예시입니다. 출력은 소스 디렉토리 밖에 둡니다.

```bash
ros2 run nav2_map_server map_saver_cli -f /tmp/prometheus-map --ros-args -p use_sim_time:=true
```

저장 파일을 확인할 때는 SLAM을 종료하거나 별도 ROS 도메인을 사용해 `/map` 발행 중복을 피합니다.
아래 명령은 각각 별도 터미널에서 같은 도메인으로 실행합니다.

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/tmp/prometheus-map.yaml
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 topic echo --once --qos-durability transient_local /map
```

지도 서버는 지도 메시지만 제공합니다. 로봇 위치 추정이나 TF를 생성하지 않습니다.
SLAM 재개에 필요한 pose graph 직렬화는 OccupancyGrid 저장과 다르며 후속 작업입니다.
