# Nav2 도착 후 YOLO+LM 제한 정렬

`src/nav2_send_goal.py`의 `--align-pallet` 경로는 `src/pallet_alignment_trial.py`를
호출한다. 기본 설정은 `config/trial.yaml`이다. 관찰용 화면과 같은 IPPE+LM 함수를
사용하고 기존 영상 신뢰도·형상·자세 검사와 속도 제어·충돌 감시 경로를 유지한다.

## 설정과 적용 범위

| 항목 | 기본값 |
|---|---:|
| 차체 앞면→팔레트 목표 간격 | 310mm |
| 포크 돌출 길이 | 152mm |
| 근접 상태에서 미정렬 시 중단 | 320mm 이하 |
| 최소 차체 간격 | 280mm |
| 전진 / 회전 명령 상한 | 10mm/s / 0.05rad/s |
| 첫 정렬 이동 명령부터 시간 제한 | 3초 |
| 정렬 시작 오도메트리 위치로부터 변위 제한 | 35mm |
| 정렬 시작 방향으로부터 회전 제한 | 0.16rad (약 9.2°) |
| 전체 정렬 단계 제한 시간 | 18초 |

시간·변위·회전 제한 중 하나에 도달하면 시험을 종료한다. 3초 동안 무조건 움직이는
것은 아니며, 조건 불충족이나 목표 도달 시 먼저 멈춘다. 310mm 정렬이 완료되기 전에
시험이 끝날 수도 있다. 제한 도달은 정렬 성공으로 보고하지 않는다.
위 제한은 정렬 단계에만 적용한다. Nav2 목적지 이동에는 --nav-timeout(기본 600초)이
별도로 적용되고 Nav2 준비 대기는 그 제한에 포함되지 않는다.
포크 삽입, 높이 조절, 들어 올리기는 수행하지 않는다.

## 개발 PC 실행

카메라·로봇 bringup·Nav2를 먼저 실행한다. 실제 정렬 전 teleop을 종료하고 다른
주행 명령을 중복 실행하지 않는다. /camera/enable 관리 서비스를 사용하는 경우에만
--enable-camera를 추가한다. 이미 카메라가 실행 중이면 생략한다.

```bash
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=11 /home/sojeong/venv/yolov3/bin/python \
  /home/sojeong/turtlebot3_ws/src/nav2_send_goal.py \
  1 --align-pallet --execute --show
```

목적지 1로 Nav2 이동 → 성공 확인 → 제한 정렬 → 정지 및 결과 저장 순서다.
Nav2 실패/취소/거부 시 정렬과 카메라 활성화 서비스는 실행하지 않는다.
정렬 설정은 Nav2 이동 시작 전에 검증한다. --alignment-config로 지정한 설정도
YOLO+LM과 시험 상한을 만족해야 한다.

관찰 모드(목적지 이동도 생략):

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
기존 `nav2_send_goal.py 1`은 여전히 실제 Nav2 이동만 수행한다.
`output/pallet_alignment_trial_20261031/run_trial.py`도 공통 실행기로 연결되며
기본 310mm 설정을 사용한다. 과거 결과 파일은 그대로 보존했다.

## 종료와 기록

Ctrl+C로 중단한다. 정렬 실행기는 종료 시 2초간 정지 명령을 기존 속도 경로로
전달하고 odom 정지를 확인한다. 관찰 모드에서는 이때도 속도를 발행하지 않는다.
결과는 `output/pallet_alignment_trials/<시각>_execute.json` 또는 `_observe.json`에
저장한다. 적용 설정, 추정 목표, 계산 명령, 이동 시작 여부, odom 변화, 종료 사유,
정지 확인 결과를 포함한다. 실행 한도가 소진되면 자동 재시도하지 않는다.

## 검증

ROS 노드 생성이나 주행 명령 없이 관련 테스트 42개 통과.
검증 범위: 기본 trial 설정 전달, Nav2 실패 시 정렬 생략, 관찰/단순 이동 모드 유지,
시간·변위·회전·속도 제한, stale/invalid odom, torque off, 활성 목표/teleop 차단,
LM 결과와 실패 처리, 종료 시 정리 및 정지 확인 실패 처리.
이번 통합 변경 후 실제 로봇 이동은 실행하지 않았다.
