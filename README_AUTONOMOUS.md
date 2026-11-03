# 실제 Waffle 자율주행

실제 Waffle 로봇의 bringup, Nav2 및 keepout 마스크 서버 실행 안내다. 경로 계획·장애물 회피·속도 제어는 Nav2가 담당하며, 지도는 사전에 작성되어 있어야 한다.

현재 작업 공간은 ROS 2 Jazzy이며 기본 환경변수는 `TURTLEBOT3_MODEL=burger`였다. 아래처럼 **로봇과 PC의 각 터미널에서** `waffle`을 지정한다. `.bashrc`는 변경하지 않았다. 이미 Burger 설정으로 실행한 bringup/Nav2는 먼저 종료한 뒤 Waffle 설정으로 다시 실행한다.

## 1. 실제 로봇의 bringup

로봇에 접속한 터미널에서 로봇에 설치된 ROS 및 작업 공간을 source한 뒤 실행한다. 로봇의 ROS 배포판과 설치 경로는 실제 설치에 맞춘다.

```bash
export TURTLEBOT3_MODEL=waffle
export ROS_DOMAIN_ID=11
ros2 launch turtlebot3_bringup robot.launch.py
```

PC와 로봇은 통신 가능한 네트워크에 있어야 하며 `ROS_DOMAIN_ID`가 같아야 한다. 직접 DDS로 연결한다면 `ROS_LOCALHOST_ONLY=1`로 제한하지 않는다. 이미 로봇에서 bringup을 실행 중이면 중복 실행하지 않는다.

## 2. PC에서 Nav2 실행

launch 소스를 수정한 뒤에는 PC 작업 공간에서 패키지를 다시 빌드한다.

```bash
source /opt/ros/jazzy/setup.bash
cd /home/sojeong/turtlebot3_ws
colcon build --packages-select turtlebot3_navigation2
```

```bash
source /opt/ros/jazzy/setup.bash
source /home/sojeong/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
export ROS_DOMAIN_ID=11
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=false \
  map:=/home/sojeong/turtlebot3_ws/map/logitle_map_fin.yaml \
  mask:=/home/sojeong/turtlebot3_ws/map/logitle_map_fin_keepout.yaml \
  params_file:=/home/sojeong/turtlebot3_ws/config/waffle_real_nav.yaml
```

위 명령은 해상도와 원점이 일치하는 `logitle_map_fin.yaml`과 keepout 마스크를 사용한다. 다른 공간이면 해당 공간에서 작성한 지도와 대응하는 마스크 YAML의 절대 경로로 교체한다. 각 YAML에 참조된 PGM도 함께 있어야 한다.

`waffle_real_nav.yaml`은 기존 Waffle 설정을 복사하고 `waypoint_follower.stop_on_failure`만 `true`로 바꾼 것이다. 실패한 경유점에서 경로 실행을 중단한다. 속도·footprint·센서값은 기존 설정 그대로이며 실물에 대한 튜닝 또는 검증을 마친 설정은 아니다. 이미 Nav2가 실행 중인 로봇을 사용한다면 중복 실행하지 말고, 그 Nav2에 같은 옵션이 적용됐는지 확인한다.

AMCL은 `waffle_real_nav.yaml`의 `initial_pose`(목적지 4)에서 시작한다. `nav2_send_goal.py`와 액션 서버는
Nav2 이동 직전마다 정지 상태의 `/scan`을 지도 벽에 맞춰(현재 AMCL 위치 기준 ±0.3m, ±30° 탐색) 가장 잘 맞는
위치를 `/initialpose`로 다시 설정하므로 RViz **2D Pose Estimate**가 필요 없다. 따라서 로봇은 초기 위치에서
0.3m·30° 안에 두고 시작한다. 일치 점수(0~1, 로그 `Localization: match a -> b`)가 0.5 미만이면 이동하지 않고
중단하며, 이때만 로봇 위치를 옮기거나 2D Pose Estimate로 설정한다. 최초 시험은 로봇 주변의 빈 공간과 짧은 목표 거리로 진행한다.

### Keepout 마스크 발행

현재 `config/waffle_real_nav.yaml`은 local/global costmap에서 keepout 필터를
사용하며 `/costmap_filter_info`를 구독한다. 위 Nav2 launch에 `mask` 경로를
전달하면 다음 세 노드를 함께 실행한다.

- `filter_mask_server`: `/keepout_filter_mask`에 마스크 발행
- `costmap_filter_info_server`: `/costmap_filter_info`에 필터 정보 발행
- `lifecycle_manager_costmap_filters`: 두 서버 자동 활성화

따라서 별도 터미널에서 `scripts/keepout.launch.py`를 실행할 필요가 없다.
기존에 별도 keepout launch를 실행했다면 먼저 종료해 중복 발행을 피한다.
`mask`를 생략하면 위 세 노드는 실행되지 않는다. keepout 필터를 활성화한
파라미터를 사용할 때는 반드시 `mask`를 전달하거나 별도 발행 서버가 있어야 한다.
시뮬레이션에서는 `use_sim_time:=true`를 추가한다.

```bash
ros2 lifecycle get /filter_mask_server
ros2 lifecycle get /costmap_filter_info_server
ros2 topic echo /costmap_filter_info --once --qos-durability transient_local
ros2 topic echo /keepout_filter_mask --once --field info --qos-durability transient_local
```

두 서버가 `active`이고 costmap 로그에 `Received filter mask`가 나타나는지
확인한다. `Filter mask was not received`가 계속되면 keepout 영역이 적용되지
않은 상태다. 마스크 이미지 로드 오류, 토픽 이름, 서버 상태 및 DDS 연결을 확인한다.

## Raspberry Pi 카메라

IMX219 카메라의 libcamera/camera_ros 구성과 Jetson 압축 영상 수신은
[카메라 실행 안내](README_CAMERA.md)를 참고한다.

## 목적지 도착 후 팔레트 정렬 시험

`src/nav2_send_goal.py`에 `--align-pallet`을 추가하면 팔레트 비전 시험을 사용할 수
있다. 이 옵션만으로는 관찰 모드이며, 실측 카메라 TF와 차체 앞면 위치를 설정한
뒤 `--execute`까지 추가해야 목적지 이동 후 정렬한다. 포크 삽입은 수행하지 않는다.
준비·실행 방법과 한계는 [팔레트 정렬 테스트 안내](README_PALLET_ALIGNMENT.md)를 참고한다.

## YOLO+LM 제한 정렬 통합 (2026-10-31)

`nav2_send_goal.py 1 --align-pallet --execute`는 Nav2 성공 후
`src/pallet_alignment_trial.py`의 제한 시험을 호출한다. 기본 설정은 `config/trial.yaml`이며,
목표 차체 간격 310mm, 포크 돌출 152mm, YOLO+LM을 사용한다.
첫 정렬 이동 명령 후 3초 / 오도메트리 변위 35mm / 회전 0.16rad 제한을 적용한다.
이 제한은 목적지까지 이동하는 Nav2 단계에는 적용되지 않는다.
실행 명령과 모드별 차이는 [제한 정렬 안내](README_PALLET_TRIAL.md)를 참고한다.
