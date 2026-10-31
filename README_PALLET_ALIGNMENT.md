> 2026-10-31 업데이트: nav2_send_goal.py의 정렬 경로는 기본 config/trial.yaml과
> 제한 시험 실행기로 통합됐다. 목표 310mm, YOLO+LM, 최대 3초/35mm/0.16rad가 적용된다.
> 최신 실행 안내는 [README_PALLET_TRIAL.md](README_PALLET_TRIAL.md)를 따른다.
> 아래 일반 정렬기·기본 contour 설정에 대한 기록과 통합 경로를 구분한다.

# 목적지 도착 후 팔레트 비전 정렬 테스트

> 2026-10-29: 학습된 `src/best.pt`를 연결하는 선택 기능을 추가했다.
> 모델용 설정, 저장 영상 평가 결과와 관찰 명령은
> [README_PALLET_MODEL.md](README_PALLET_MODEL.md)를 참고한다.
> 아래 인식 방식 설명은 기본 `detector: contour` 설정에 해당한다.
>
> 추가 실측 반영: 중앙 받침 폭 21.248mm, 차체 앞면→포크 끝 약 148mm.
> 사용자 요청에 따라 정렬 목표 170mm, 최소 간격 160mm, 근접 정렬 제한
> 190mm를 두 설정 파일에 반영했다. 포크 끝의 목표 여유는 약 22mm이다.
> 전체 치수는 config/pallet_mechanical_spec.yaml에 기록했다.

이 코드는 **포크 규격이 확정되기 전의 시험용**이다. Nav2 목적지 이동 성공 후
팔레트의 두 입구를 검출하고, 차체 앞면과 팔레트 앞면 사이 170mm에서 정렬을
마친다. 포크 삽입·리프트 동작은 하지 않는다. 실제 장비에서의 검출률과 정렬
정확도는 아직 검증하지 않았다.

## 이번에 추가한 것과 실행 위치

개발 PC의 `/home/sojeong/turtlebot3_ws`에서 아래 파일을 작성했다.
패키지 설치, 사용자 권한 변경, 원격 장비 배포, 카메라 재시작, 실물 이동은
이번 코드 작성 과정에서 수행하지 않았다. 아래 원격 복사·실행 명령은 사용자가
시험할 때 수행하는 절차이며 이미 수행한 작업을 뜻하지 않는다.

| 파일 | 역할 / 실행 위치 |
| --- | --- |
| `src/nav2_send_goal.py` | 목적지 번호 선택 → Nav2 성공 확인 → 정렬, PC 또는 Jetson |
| `src/pallet_vision.py` | 입구 윤곽 검출·PnP 자세 추정·정렬 제어 계산 |
| `src/pallet_alignment.py` | JPEG/CameraInfo 구독, TF 변환, 관찰/실제 정렬 |
| `config/pallet_alignment.yaml` | 실측 치수, 검출 영역, 속도 및 정지 조건 |
| `scripts/waffle_camera_manager.py` | Pi에서 `/camera/enable` 서비스로 카메라 프로세스 관리 |
| `tests/test_pallet_alignment.py` | 로봇 없이 실행하는 합성 영상·제어·실패 경로 테스트 |

기존 목적지 좌표는 변경하지 않았다. `nav2_send_goal.py`의 정의되지 않은 `z`
참조는 `0.0`으로 수정했다. 2·3·4번 좌표는 여전히 미설정이다.

## 인식 방식과 한계

마커나 학습된 YOLO 모델을 사용하지 않는다. 지정한 ROI 안에서 어두운 사각형
윤곽 두 개를 찾고, 입구의 폭·높이·간격으로 모양을 검사한다. 두 입구의 **앞면
모서리 8개**와 실제 치수를 대응시켜 OpenCV IPPE/PnP로 자세를 추정한다.
원본 JPEG 좌표에 CameraInfo의 K/D를 적용한다. 미리 왜곡 보정한 영상을 같은
설정으로 넣으면 안 된다.

- 입구 폭 35.158mm, 높이 9.165mm, 중앙 받침 폭 21.248mm를 사용한다.
- 따라서 두 입구 중심 간격은 56.406mm이다. 수치는 CAD/실측 입력값이며 이
  소수점 자릿수만큼 실제 인식 정확도가 보장되는 것은 아니다.
- 바닥 그림자와 입구가 붙거나, 입구 아래/뒤쪽 윤곽을 앞면 모서리로 잡으면
  자세가 잘못 추정될 수 있다. 박스·체커보드·다른 검은 사각형도 오검출 후보가 된다.
- 가까운 두 자세 해가 서로 다른 방향을 나타내거나, 유효한 입구 쌍이 여러 개면
  검출을 거부한다. 이것만으로 모든 오검출을 제거하지는 못한다.
- 두 입구가 모두 보이는 거의 정면 접근용이다. 입구가 너무 작게 보이거나
  심하게 기울거나 가려진 경우를 처리하는 학습 모델은 포함하지 않는다.
- 화면의 녹색 윤곽이 **팔레트 전면 입구의 실제 경계**와 일치하는지 반드시
  관찰 모드로 확인한다. 재투영 오차가 작다는 사실만으로 실제 거리 정확도를
  판정할 수 없다. 줄자로 잰 여러 거리·각도와 비교해야 한다.

알고리즘 참고: [OpenCV PnP 공식 설명](https://docs.opencv.org/4.10.0/d5/d1f/calib3d_solvePnP.html).

## 1. 현재 바로 할 수 있는 관찰 시험 — 로봇 이동 없음

카메라가 이미 실행 중인 상태에서 개발 PC의 그래픽 터미널에서 실행한다.
ROS 환경은 해당 터미널에만 적용한다. 기존 NumPy 2 / apt OpenCV 충돌을 피하기
위해 명령별로 Ubuntu 배포판 Python 패키지를 우선한다. pip 설치나 삭제는 하지 않는다.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=11
PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH \
  /usr/bin/python3 /home/sojeong/turtlebot3_ws/src/pallet_alignment.py --show
```

이 명령은 Nav2 목표나 속도 명령을 보내지 않는다. GUI가 없는 SSH 터미널에서는
`--show`를 빼고 로그만 확인한다. 종료는 `q` 또는 Ctrl+C.

노란 사각형이 탐색 ROI, 녹색 두 사각형이 검출된 입구이다. 화면의 `z`는
**카메라 광학 좌표계의 전방 거리**이며 차체 앞면과의 거리가 아니다.
카메라 TF가 미확인인 현재 설정에서는 차체 거리나 실제 제어 명령을 계산하지 않는다.

검출이 안 되면 `/home/sojeong/turtlebot3_ws/config/pallet_alignment.yaml`을 수정한
뒤 다시 실행한다.

- `roi: [left, top, right, bottom]`: 영상 크기에 대한 0~1 비율. 두 입구만
  포함하고 박스/배경 체커보드를 제외한다. 입구 윤곽이 ROI 경계에 닿으면 거부한다.
- `dark_threshold`: 이 밝기 이하를 입구 후보로 본다. 초기값 65.
- `min_opening_height_px`: 입구가 너무 작게 보이면 추정을 거부한다. 초기값 8px.
  작은 입구를 억지로 검출하려고 낮추기보다 카메라 시야와 접근 위치부터 확인한다.

기본 15초 이내 안정된 검출이 없거나, 검출 후 2초 이상 잃으면 종료한다.
전체 시험은 최대 90초다. 필요하면 관찰 시험 중 설정 파일의 제한 시간을 조정한다.
영상 시각도 검사하므로 PC/Pi/Jetson의 시스템 시간이 동기화되어 있어야 한다.

목적지 인자를 포함한 관찰 시험도 가능하다. **이 경우에도 이동하지 않는다.**

```bash
PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH \
  /home/sojeong/venv/yolov3/bin/python /home/sojeong/turtlebot3_ws/src/nav2_send_goal.py \
  1 --align-pallet --show
```

## 2. 카메라를 소스로 켜는 선택 구성

카메라가 이미 실행 중이면 이 단계 없이 관찰 시험을 할 수 있다.
도착했을 때 카메라를 켜려면 **Pi의 카메라 관리자 노드**를 미리 실행한다.
PC의 Python 프로세스에서 로컬 카메라 실행 파일을 호출하는 방식이 아니다.

개발 PC에서 새 소스 파일을 Pi의 기존 scripts 폴더로 복사:

```bash
scp /home/sojeong/turtlebot3_ws/scripts/waffle_camera_manager.py \
  waffle@192.168.0.105:/home/waffle/camera_ws/scripts/
```

Pi 터미널에서 실행:

```bash
source /opt/ros/jazzy/setup.bash
source /home/waffle/camera_ws/install/setup.bash
ROS_DOMAIN_ID=11 /usr/bin/python3 \
  /home/waffle/camera_ws/scripts/waffle_camera_manager.py \
  --params-file /home/waffle/camera_ws/config/waffle_camera.yaml
```

`source`는 기존 설치 환경을 읽는 준비 과정이다. 카메라 실행 자체는 Python의
`subprocess.Popen([...])`이 `camera_ros` 실행 파일을 직접 시작한다. 별도의
`.sh` 파일, sudo, chmod, pip/apt 설치는 사용하지 않는다. 보정 파일 경로 등은
기존 `waffle_camera.yaml`을 그대로 읽는다.

관리자가 실행된 뒤 관찰/미션 명령에 `--enable-camera`를 붙이면
`/camera/enable` (`std_srvs/srv/SetBool`)에 `data=true`를 요청한다.
기존 카메라 발행자가 보이면 재사용하여 중복 실행을 피한다. 시작 응답 후 실제
영상 도착 여부는 정렬 노드가 검사한다. 관리자는 자신이 시작한 프로세스만
종료할 수 있고 외부에서 실행한 카메라에는 종료 신호를 보내지 않는다.

서비스를 직접 시험하려면 Pi 또는 PC의 도메인 11 터미널에서:

```bash
ros2 service call /camera/enable std_srvs/srv/SetBool '{data: true}'
# 관리자가 직접 시작한 카메라만 종료
ros2 service call /camera/enable std_srvs/srv/SetBool '{data: false}'
```

정렬 완료 후 카메라는 유지한다. 관리자 Ctrl+C 종료 시 자신이 시작한 카메라에
SIGINT를 보낸다. 종료가 지연되면 PID를 출력하며 무조건 kill하지 않는다.
부팅 자동 실행은 이번에 설정하지 않았다.

## 3. 실제 정렬 전에 측정할 것

현재 실제 이동 옵션은 아래 두 설정이 없으면 **목적지로 출발하기 전** 거부한다.

1. 실제 IMX219의 광학 좌표계와 `base_link` 사이 위치·회전을 측정하여 TF를 맞춘다.
   카메라 TF는 2026-10-29 Waffle Pi/IMX219 기본값으로 교체했으며,
   광학 원점은 base_link 기준 [76, 0, 93]mm다. 실측 장착 오차는 별도 확인한다.
   기존 프레임에 다른 static TF를 중복 발행하지 말고 해당 장착 TF를 수정해야 한다.
2. `base_link` 원점에서 **차체 앞면까지의 x방향 거리(m)**를 재서
   사용자 실측 68mm를 `body_front_x: 0.068`로 반영했다.
   약 148mm인 포크 돌출 길이는 이 값에 넣지 않는다.
3. 실제 TF와 전면 기준을 검증한 뒤에만 `extrinsics_verified: true`로 바꾼다.

설정 예시 수치를 제공하지 않는 이유는 현재 실측 위치가 없기 때문이다.
확인하지 않고 플래그만 바꾸면 170mm 정지 기준이 잘못 적용될 수 있다.

포크 끝은 차체 앞면에서 약 148mm 돌출된다. 목표 170mm에서 입구 앞 여유는
약 22mm이며, 5mm 정지 허용 오차의 가까운 쪽 경계에서는 약 17mm이다.
최소 차체 간격은 160mm로 설정했다. `target_body_gap - gap_tolerance`와
`min_body_gap` 모두 포크 돌출
길이보다 커야 한다. 이 검사는 정적인 기하 조건이며 정지 거리·실측 오차 검증을
대신하지 않는다.
현재 정렬 허용값 8mm/4도는 초기 시험용이며 포크 삽입 허용 오차가 아니다.
119mm 깊이 및 원하는 115mm 삽입 거리는 제어에 사용하지 않는다.

## 4. 목적지 이동 → 카메라 → 실제 정렬

실측 설정과 관찰 시험을 마친 뒤 실행하는 명령이다. **실제 로봇이 움직인다.**
Nav2/bringup을 기존 안내대로 실행하고 AMCL 위치를 맞춰 둔다. 1번 목적지는
팔레트 전면을 볼 수 있고 정렬할 공간이 남는 접근 지점이어야 한다. 이 코드는
1번 좌표가 그런 위치인지 지도만으로 판정하지 않는다. 처음부터 170mm 앞에
목표를 두지 말고, 입구가 보이는 더 먼 거리에서 정렬을 시작한다.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=11
PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH \
  /home/sojeong/venv/yolov3/bin/python /home/sojeong/turtlebot3_ws/src/nav2_send_goal.py \
  1 --align-pallet --execute --enable-camera --show
```

이미 카메라가 실행 중이고 관리자를 사용하지 않으면 `--enable-camera`를 생략한다.
Jetson에서 처리하려면 `src`의 세 Python 파일과 `config/pallet_alignment.yaml`을
같은 상대 구조로 복사하고, 위 절대 경로를 Jetson 복사 위치로 바꿔 실행한다.
Pi에서 JPEG를 발행하고 Jetson에서 디코딩·PnP·정렬 계산을 수행할 수 있다.

현재 위치에서 정렬만 시험하려면:

```bash
PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH \
  /usr/bin/python3 /home/sojeong/turtlebot3_ws/src/pallet_alignment.py \
  --execute --show
```

기존 `python3 src/nav2_send_goal.py 1`의 **Nav2 이동만 하는 동작은 유지**한다.
`--align-pallet`을 붙인 경우에만 `--execute`가 실제 이동 스위치로 동작한다.

## 동작 중 정지 조건과 속도 경로

Nav2 `SUCCEEDED`일 때만 카메라 시작/정렬로 넘어간다. 이동 실패·취소·목표 거부 시
종료한다. 미션 Ctrl+C 시 진행 중인 Nav2 목표를 취소하고, 정렬 중에는 0속도를
발행한다. Nav2 이동 기본 제한은 600초이며 Nav2 활성화 대기 시간은 포함하지 않는다.

정렬 최대 속도는 전진 0.025m/s, 회전 0.15rad/s이다. 영상 지연, TF 실패,
보정 정보 불일치, 검출 실패, 자세 급변 시 0속도를 보낸다. 8개 연속 프레임이
일관된 경우에만 제어를 시작한다. 완료 조건도 8개 연속 프레임에서 확인한다.
차체 간격이 190mm 이하인데 좌우/각도 정렬이 불충분하면 `NEED_REPOSITION`으로
멈춘다. 자동 후진·재진입·포크 삽입은 없다.

속도는 현재 Nav2 구성에 맞춰 다음 경로로 전달한다.

```text
정렬 노드 TwistStamped /cmd_vel_nav
  → velocity_smoother → /cmd_vel_smoothed
  → collision_monitor → /cmd_vel → Waffle 구동부
```

시작 전에 메시지 형식·구독 경로·일부 파라미터와 두 노드의 active 상태를 검사한다.
동작 중 lifecycle 상태도 확인한다. `/cmd_vel`에 직접 발행하거나 collision monitor를
우회하지 않는다. 실제 센서의 사각지대까지 이 검사가 보장하지는 않는다.
다른 RViz 목표/teleop/도킹 작업과 동시에 실행하지 않는다. 이 프로토타입에는
여러 제어기의 속도 명령을 중재하는 별도 mux가 없다.

프로세스 강제 종료나 네트워크 단절 시에는 Pi/구동부의 기존 명령 시간 초과
정지가 필요하다. 현재 설정의 smoother `velocity_timeout`은 1초이다. 실물 시험
전에 이 경로와 구동부 정지를 확인해야 한다. 코드가 로봇 설정을 자동 변경하지 않는다.

## 로봇 없이 검증

개발 PC에서 수행한 명령:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/sojeong/turtlebot3_ws
PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH \
  /usr/bin/python3 -m unittest discover -s tests -p 'test_pallet_alignment.py' -v
```

합성 입구 영상/PnP, 여러 후보 거부, 카메라→차체 거리 변환, 소규모 횡방향 오차의
차동구동 수치 모의시험, 근접 정지, 자세 급변, 실측 설정 요구, 오래된 영상,
Nav2 실패/취소/시간 초과, 관찰 모드의 이동 차단을 검증한다.
이 결과는 실제 팔레트 검출 성공이나 170mm 실물 정지 정확도를 증명하지 않는다.
