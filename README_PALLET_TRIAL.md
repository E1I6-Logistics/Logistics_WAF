# 목적지 이동 후 팔레트 고정 목표 정렬

`src/nav2_send_goal.py`의 `--align-pallet` 경로는 `src/pallet_alignment_trial.py`를
호출한다. 기본 설정은 `config/trial.yaml`이다. `--execute`를 주면 Nav2로 지정한 목적지까지
이동하고, 성공(SUCCEEDED)했을 때만 그 자리에서 정렬을 시작한다. 목적지 좌표는
`nav2_send_goal.py`의 `DESTINATIONS`에 있다.
정렬이 확인되면(`--execute`) 포크를 넣기 위해 그대로 직진하고, 리프트업 → 100mm 직선 후진 →
목적지 2로 Nav2 이동 → 리프트다운 → 150mm 후진 순서로 팔레트를 옮긴다. 이어서 목적지 3 앞의
팔레트를 같은 방식으로 들어 첫 팔레트가 있던 자리에 내려놓고 목적지 5를 거쳐 목적지 4로 이동한다.

## 동작

팔레트는 정렬 중 움직이지 않는다고 보고, 주행 중 영상 추정값을 따라가지 않는다.

1. **정지 관측**: odom으로 정지(5mm/s, 0.02rad/s 미만)를 확인한 상태의 새 영상
   8프레임을 odom 좌표의 팔레트 중심·방향으로 변환한다. 8개가 모두 중앙값에서
   15mm·3° 안에 있어야 그 중앙값을 고정 목표로 쓴다. 이동 중이나 정지 확인 전에
   촬영된 영상은 쓰지 않는다. 12초 안에 얻지 못하면 중단한다.
2. **판정**: 고정 목표와 현재 odom으로 차체 간격, 방향 오차, **앞면 좌우 오차**를
   계산한다. 앞면 좌우 오차는 차체 정면 축(포크 진행선)이 팔레트 앞면과 만나는 점과
   입구 중심 사이 거리다. 간격 310±7mm, 앞면 좌우 ±5mm, 방향 ±3°를 모두 만족하면 완료한다.
   (포크 한쪽 옆 여유는 약 4.4mm다.)
3. **이동 계획** (더 엄격한 기준: 간격 ±5mm, 앞면 좌우 ±3mm, 방향 ±1.5°):
   - base_link가 진입 중심선에서 8mm 넘게 벗어나면 중심선 위 350mm 지점으로
     회전 → 직진 → 팔레트 정면 회전. 그 지점을 45° 이내로 갈 공간이 없으면 정면으로
     회전 후 410mm까지 직선 후진.
   - 그 안이면 필요 시 **입구 중심을 향해** 회전한 뒤 310mm까지 직진(가까우면 후진).
     직진해도 포크 진행선은 입구 중심을 지나므로 앞면 좌우 오차가 유지된다.
4. **이동**: odom 기준으로 회전(0.05rad/s, 3° 이내 0.01rad/s, ±0.3°에서 정지),
   직진·후진(10mm/s)하고 정지를 0.5초 확인한다. 이후 1로 돌아가 다시 관측한다.
   관측은 최대 15회이며, 1회 직진·후진 계획이 250mm를 넘으면 실행하지 않는다.

5. **삽입**: 완료 판정 후 `--execute`이면 회전 없이 `관측 간격 − fork_projection(152mm)
   + insertion_depth(105mm)`, 즉 약 263mm를 `insertion_speed`(40mm/s)로 직진한다. 입구가 너무 가까워 카메라로
   보지 않고 odom 거리만 쓴다. 속도 경로는 정렬과 같다(collision_monitor 경유).
   실물 돌출 길이가 152mm보다 길면 그만큼 깊이 들어간다(내부 장애물 119mm).
6. **리프트업·후진**: `/linear_actuator/cmd`에 `UP`을 보내고 `lift_time`(10초) 뒤 `STOP`.
   액추에이터에 위치 센서가 없으므로 최대 높이 도달은 시간으로 판단한다(전체 행정 시간 + 여유로
   실측해 설정). 이후 `retreat_distance`(100mm)를 40mm/s로 직선 후진한다(후방 스캔 검사 적용).
7. **운반·하강**: `nav2_send_goal.py`가 목적지 2 `DESTINATIONS[2]`로 Nav2 이동하고, 성공하면
   `DOWN`을 11초 동안 보낸 뒤 `STOP`한다. 이동 실패 시 내리지 않고 종료한다.
8. **후진**: Nav2 BackUp 동작으로 150mm(25mm/s) 후진한다(포크 삽입 깊이 105mm).
9. **두 번째 회수**: 목적지 3 `DESTINATIONS[3]`로 이동해 1~6과 같이 정렬·삽입·리프트업·100mm 후진한다.
10. **제자리 놓기**: 첫 정렬 완료 시점(삽입 직전)의 map 좌표(AMCL, `map→base_link` TF)와 삽입 거리를
    메모리에 저장해 둔다(odom은 이후 주행 중 누적 오차가 생기므로 쓰지 않는다). 그 좌표로 Nav2 이동한 뒤
    리프트다운(11초)하고, 정지 상태에서 map 좌표를 한 번만 읽어 저장 좌표를 odom 좌표로 바꾸고, 이후 보정은 odom만으로 한다
    (짧은 보정 이동 사이 AMCL 값이 약 10mm·1.5°씩 튀기 때문). 좌우 오차가 20mm를 넘으면 그 점을 향해
    (최대 90° 회전) 회전·직선 이동한 뒤 저장한 방향으로 회전하고, 좌우 20mm·방향 3° 안에 들 때까지
    최대 6회 반복한다(`place_lateral_tolerance`, `place_yaw_tolerance_deg`). 남은 앞뒤 오차를 더한
    삽입 거리만큼 40mm/s로 직진하고, 150mm 직선 후진 → 목적지 5 `DESTINATIONS[5]` → 목적지 4 `DESTINATIONS[4]`로 이동한다.
    정밀도는 AMCL 위치 추정에 좌우된다. 1회 보정 거리가 250mm를 넘으면 중단한다.

가까이 갈수록 방향 추정이 정확해지므로, 먼 위치의 첫 관측 오차는 다음 정지 관측에서
보정된다. 첫 추정의 일관된 편향이나 바퀴 미끄러짐은 마지막 정지 관측으로만 확인된다.

## 자세 추정

YOLO Pose 8점 → IPPE+LM으로 초기값 → **앞면을 수직으로 고정한 4자유도(x, y, z, yaw)
재추정**. 8점만으로는 앞면 상하 기울기가 거의 결정되지 않고, 기울기가 자유로우면
yaw와 맞바뀌어 흔들린다. IPPE의 두 후보는 주로 기울기만 다르며 수직 고정 후 같은 yaw로
수렴하므로, 이 경로(`vertical_face_fit: true`)에서는 후보 모호성으로 거부하지 않는다.
재추정 재투영 오차가 1.5px를 넘으면 그 프레임은 버린다.

PnP용 `opening_height`는 실물 9.165mm가 아닌 **10.5mm**다. 학습 라벨의 입구 높이/폭
비율이 0.299로 실물(0.261)보다 커서(모서리를 입구 바깥쪽 약 1px에 찍은 라벨 기준),
실물 값을 쓰면 PnP가 약 17° 기울기를 만들어냈다. 폭·기둥 폭은 실물과 1% 안에서 맞는다.
실물 치수 기록은 `config/pallet_mechanical_spec.yaml`에 있다.

## 안전 조건

- 속도: 정렬 중 전후진 0.01m/s, 삽입·리프트 후 후진 0.04m/s, 회전 ≤0.05rad/s. OpenCR은 `int(v*100)`을 받으므로 0.01 미만은 0이 된다.
- 명령 경로 `/cmd_vel_nav → velocity_smoother → collision_monitor → /cmd_vel`, 두 노드 active 확인.
- 최신 `/odom`·`/sensor_state`(0.5초 이내), 모터 토크, 다른 Nav2 동작·teleop 없음. Wi-Fi 등으로 odom·sensor_state가
  0.5초 넘게 끊기면 정지 명령을 보내며 최대 2초 회복을 기다린 뒤 이어서 진행하고, 회복되지 않으면 중단한다.
- 후진은 회전 없이 직선으로만. 0.3초 이내 `/scan`과 scan→base TF로 차체 뒤
  x=-400~0mm, 좌우 ±200mm에 반사점이 없어야 한다. smoother의 음수 속도 허용·deadband 확인.
- 관측 간격 280mm 미만이면 중단. 전체 300초 제한(삽입은 별도 제한).
- 직진·후진 중 odom 진행이 3초간 1mm 미만이면(포크 걸림, collision_monitor 정지 등) 정지하고 중단.

## 개발 PC 실행

`--execute`에서는 먼저 기존 AMCL 위치를 사용해(초기 위치를 다시 보내지 않음) Nav2 목적지로
이동한다. 이동이 실패·거부·취소되면 정렬과 카메라 호출로 넘어가지 않는다.
정렬 계산은 카메라→차체 TF와 odom을 사용하며 목적지 좌표는 사용하지 않는다.
실제 정렬은 활성 상태의 `velocity_smoother`와 `collision_monitor`,
최신 `/odom`·`/sensor_state`, 후진 시 `/scan`, 켜진 모터 토크를 요구한다. 기존 Nav2 bringup을
사용할 때는 지도·keepout·코스트맵에 필요한 위치 추정과 TF도 정상이어야 한다.

카메라·로봇 bringup·Nav2를 먼저 실행한다. 실제 정렬 전 teleop을 종료하고 다른
주행 명령을 중복 실행하지 않는다. /camera/enable 관리 서비스를 사용하는 경우에만
--enable-camera를 추가한다. 이 경우 정렬(ALIGN, ALIGN_2)마다 직전에 켜고 끝나면(성공·실패·Ctrl+C) 끈다. 매니저가 직접 켠
카메라만 꺼지므로, 카메라를 따로 실행해 두면 생략하고 직접 종료한다.

```bash
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=11 /home/sojeong/venv/yolov3/bin/python \
  /home/sojeong/turtlebot3_ws/src/nav2_send_goal.py \
  1 --align-pallet --execute --show
```

목적지 1로 Nav2 이동 → 첫 영상으로 모델·화면 초기화 → 센서 재확인 → 정지 관측·이동 반복 → 완료 또는 중단 → 정지 및 결과 저장 순서다.
도착 시 팔레트 입구가 카메라 관심 영역 안에 보이는 위치·방향으로 목적지를 정해야 한다.
첫 추론은 후속 추론보다 오래 걸릴 수 있으므로 속도 발행기를 만들기 전에 준비한다.
준비에 사용한 검출 결과는 버린다. --alignment-config로 지정한 설정도
시험 시작 전에 같은 검증을 통과해야 한다.

관찰 모드(목적지 이동 없음. 현재 위치에서 1회 정지 관측과 이동 계획만 출력, 속도 발행 없음. `/odom` 필요):

```bash
ROS_DOMAIN_ID=11 /home/sojeong/venv/yolov3/bin/python \
  /home/sojeong/turtlebot3_ws/src/nav2_send_goal.py 1 --align-pallet --show
```

현재 위치에서 정렬만 시험(Nav2 목적지 이동 생략):

```bash
ROS_DOMAIN_ID=11 /home/sojeong/venv/yolov3/bin/python \
  /home/sojeong/turtlebot3_ws/src/pallet_alignment_trial.py --execute --show
```

`--execute`를 빼면 관찰만 하며 속도 publisher를 만들지 않는다.
`nav2_send_goal.py 1`만 실행하면 목적지 이동만 하고 정렬하지 않는다.
`output/pallet_alignment_trial_20261031/run_trial.py`도 공통 실행기로 연결되며
기본 310mm 설정을 사용한다. 과거 결과 파일은 그대로 보존했다.

## 액션 서버 실행

`src/pallet_replace_server.py`는 위 `--execute` 전체 순서를 액션 `/pallet_replace`
(`waffle_interfaces/action/PalletReplace`)로 실행한다. goal은 출발 목적지 번호만 받는다. 목적지 도착 후 정렬 직전에 `/camera/enable`로 카메라를
켜고, 정렬이 끝나면(성공·실패·취소 모두) 끈다. Nav2 주행 중에는 꺼 둔다. Waffle에서 카메라 매니저(`scripts/waffle_camera_manager.py`)가
실행 중이어야 한다. 매니저가 직접 켠 카메라만 끄며, 끄기에 실패해도 작업 결과는 바뀌지 않고 경고만 남긴다.
카메라 영상(OpenCV) 창은 띄우지 않고, 한 번에 하나의 goal만 받는다.
feedback `stage`는 NAV_TO_PICK, ALIGN(정렬·삽입·리프트업·후진), NAV_TO_DROP, LIFT_DOWN, BACKUP,
NAV_TO_PICK_2, ALIGN_2, NAV_TO_PLACE, LIFT_DOWN_2, PLACE(보정·직진), BACKUP_2, NAV_TO_VIA(목적지 5), NAV_TO_HOME 순이다.
목적지 2·3·4는 `DESTINATIONS` 고정값이고, 놓을 좌표는 같은 goal 안에서만 메모리로 유지한다.
취소하면 정지 명령·Nav2 취소·리프트 STOP 후 CANCELED로 끝난다.

```bash
cd /home/sojeong/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select waffle_interfaces   # 최초 1회 또는 .action 변경 시
source install/setup.bash
ROS_DOMAIN_ID=11 /home/sojeong/venv/yolov3/bin/python src/pallet_replace_server.py
```

```bash
ROS_DOMAIN_ID=11 ros2 action send_goal /pallet_replace waffle_interfaces/action/PalletReplace "{destination: 1}" --feedback
```

액션을 호출하는 쪽(FMS 등)에도 `waffle_interfaces` 패키지를 빌드해 source해야 한다.
`trial.yaml`은 서버 시작 시 한 번 검증하므로 수정 후 서버를 재시작한다.

## 종료와 기록

Ctrl+C로 중단한다. 종료 시 2초간 정지 명령을 전달하고 odom 정지를 확인한다.
관찰 모드에서는 이때도 속도를 발행하지 않는다. 결과는
`output/pallet_alignment_trials/<시각>_execute.json` 또는 `_observe.json`에 저장한다.
프레임별 `target`, 관측별 `pallet_odom`·`errors`·`moves`, 이동 명령과 odom, 종료 사유,
정지 확인 결과를 포함한다.

## 검증

기록된 정지 영상 240프레임에 새 추정을 적용해 모두 통과했다(약 480mm에서 yaw 표준편차
3.6°→0.8°, 범위 16°→3.4°). 잡음 0.3px 운동 시뮬레이션에서 6개 시작 자세 모두 2~4회
관측 안에 완료했다(최종 중심선 ±3mm, 방향 ±0.6° 이내). 실제 로봇 주행과 실제 팔레트
방향 대비 yaw 절대값 검증은 아직 하지 않았다.
