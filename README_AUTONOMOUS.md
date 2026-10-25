# 실제 Waffle 자율주행

실제 Waffle 로봇의 bringup, Nav2 및 keepout 마스크 서버 실행 안내다. 경로 계획·장애물 회피·속도 제어는 Nav2가 담당하며, 지도는 사전에 작성되어 있어야 한다.

현재 작업 공간은 ROS 2 Jazzy이며 기본 환경변수는 `TURTLEBOT3_MODEL=burger`였다. 아래처럼 **로봇과 PC의 각 터미널에서** `waffle`을 지정한다. `.bashrc`는 변경하지 않았다. 이미 Burger 설정으로 실행한 bringup/Nav2는 먼저 종료한 뒤 Waffle 설정으로 다시 실행한다.

## 1. 실제 로봇의 bringup

로봇에 접속한 터미널에서 로봇에 설치된 ROS 및 작업 공간을 source한 뒤 실행한다. 로봇의 ROS 배포판과 설치 경로는 실제 설치에 맞춘다.

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_bringup robot.launch.py
```

PC와 로봇은 통신 가능한 네트워크에 있어야 하며 `ROS_DOMAIN_ID`가 같아야 한다. 직접 DDS로 연결한다면 `ROS_LOCALHOST_ONLY=1`로 제한하지 않는다. 이미 로봇에서 bringup을 실행 중이면 중복 실행하지 않는다.

## 2. PC에서 Nav2 실행

```bash
source /opt/ros/jazzy/setup.bash
source /home/sojeong/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=false \
  map:=/home/sojeong/turtlebot3_ws/my_map.yaml \
  params_file:=/home/sojeong/turtlebot3_ws/config/waffle_real_nav.yaml
```

`my_map.yaml`은 작업 공간에 있던 지도이며 현재 실물 환경과 일치하는지는 확인되지 않았다. 다른 공간이면 해당 공간에서 작성한 지도 YAML의 절대 경로로 교체한다. 지도에 참조된 PGM도 함께 있어야 한다.

`waffle_real_nav.yaml`은 기존 Waffle 설정을 복사하고 `waypoint_follower.stop_on_failure`만 `true`로 바꾼 것이다. 실패한 경유점에서 경로 실행을 중단한다. 속도·footprint·센서값은 기존 설정 그대로이며 실물에 대한 튜닝 또는 검증을 마친 설정은 아니다. 이미 Nav2가 실행 중인 로봇을 사용한다면 중복 실행하지 말고, 그 Nav2에 같은 옵션이 적용됐는지 확인한다.

RViz의 **2D Pose Estimate**로 로봇의 실제 위치와 방향을 지정하고, LaserScan이 지도 벽과 맞는지 확인한다. AMCL 메시지 수신만으로 위치 정확도를 판정할 수 없으므로 이 확인이 필요하다. 최초 시험은 로봇 주변의 빈 공간과 짧은 목표 거리로 진행한다.

### Keepout 마스크 발행

현재 `config/waffle_real_nav.yaml`은 local/global costmap에서 keepout 필터를
사용하며 `/costmap_filter_info`를 구독한다. 기본 navigation2 launch는 필터용
마스크 서버를 실행하지 않는다. 별도의 서버가 없다면 새 터미널에서 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch /home/sojeong/turtlebot3_ws/scripts/keepout.launch.py
```

이 launch는 `map/logitle_map_fin_keepout.yaml`을 `/keepout_filter_mask`로
발행하고, `/costmap_filter_info` 발행 서버와 두 서버의 lifecycle manager를
함께 실행한다. Nav2와 같은 `ROS_DOMAIN_ID`를 사용하고 터미널을 유지한다.
기존 마스크 서버가 있다면 중복 실행하지 않는다. 다른 마스크는
`mask:=/absolute/path/to/mask.yaml`로 지정하고, 시뮬레이션에서는
`use_sim_time:=true`를 추가한다. 마스크의 좌표는 Nav2에서 사용하는 지도와
일치해야 한다. 기본 마스크는 `map/logitle_map_fin.yaml`과 해상도 및 원점이 같다.

```bash
ros2 lifecycle get /keepout_mask_server
ros2 lifecycle get /costmap_filter_info_server
ros2 topic echo /costmap_filter_info --once --qos-durability transient_local
ros2 topic echo /keepout_filter_mask --once --field info --qos-durability transient_local
```

두 서버가 `active`이고 costmap 로그에 `Received filter mask`가 나타나는지
확인한다. `Filter mask was not received`가 계속되면 keepout 영역이 적용되지
않은 상태다. 마스크 이미지 로드 오류, 토픽 이름, 서버 상태 및 DDS 연결을 확인한다.
