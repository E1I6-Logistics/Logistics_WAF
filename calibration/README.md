# Waffle IMX219 카메라 보정

2026-10-27 촬영 영상 109장으로 계산한 보정값을 저장하고 Pi에 적용했다.
재시작 후 `/camera/camera_info`의 640×480 크기 및 K/D/R/P 행렬이 파일과
일치함을 확인했다. 보정 창 종료 후 일반 통신 설정으로 재시작해
Pi 원본 영상과 Jetson 압축 영상 수신도 정상임을 확인했다.
이 절차는 렌즈 왜곡과 카메라 내부 파라미터를 구하는 단안 보정이며,
카메라와 로봇 본체 사이의 TF 위치 보정은 별도다.

## 적용 결과 (2026-10-27)

- 최종 보정: [imx219_640x480_20261027.yaml](imx219_640x480_20261027.yaml)
- GUI가 저장한 원본: [ost_20261027_original.yaml](ost_20261027_original.yaml)
- 수집 영상 검사: [validation_20261027.json](validation_20261027.json)
- 재시작 후 실시간 확인: [runtime_verification_20261027.json](runtime_verification_20261027.json)
- 촬영 원본: `captures/calibrationdata_20261027_161551.tar.gz` (Git 제외)

원본 압축 파일 SHA256:
`99957a8a9da14ba860ed49719ef9ac67d463cd42e5badfcc838d13c85cf9d096`

최종 YAML은 원본의 K/D/R/P 숫자를 유지하고 `camera_name`만 실제 드라이버 이름으로
맞췄다. 해상도는 640×480, sensor_mode는 1640×1232이며 보정 모델은 `plumb_bob`이다.
내부 교차점 8×6 검출은 109장 모두 성공했다. 저장한 K/D를 고정하고 각 영상의
자세만 다시 추정한 재투영 RMS는 0.424픽셀, 영상별 RMS의 95백분위는 0.694픽셀,
최댓값은 0.773픽셀이다. 직선성 오차 중앙값은 0.673에서 0.329픽셀로 감소했다.
이는 수집에 사용한 영상에 대한 재검사이며 별도 촬영한 검증 세트의 성능은 아니다.
자세 추정에는 대화에서 정한 한 칸 0.02875m를 사용했고 실물 치수는 독립 측정하지 않았다.

Pi에 적용한 경로:

```text
/home/waffle/camera_ws/config/imx219_640x480_20261027.yaml
/home/waffle/camera_ws/config/waffle_camera.yaml
```

`waffle_camera.yaml`에 다음 항목을 추가했다.

```yaml
camera_info_url: "file:///home/waffle/camera_ws/config/imx219_640x480_20261027.yaml"
```

이전 설정 백업은 Pi의
`/home/waffle/camera_ws/config/waffle_camera.before_calibration_20261027.yaml`이다.
카메라 종료 중 송신 스레드가 대기해 정상 종료가 끝나지 않았으므로 확인된 카메라
PID만 강제 종료 후 재시작했다. ROS 파서의 `convert`로 YAML 읽기도 성공했다.

**최종 통신 확인:** 사용자가 보정 창을 닫은 뒤 카메라를 일반 통신 설정으로
재시작했다. Pi에서 10초간 원본 영상 146프레임(14.61 FPS)을 수신했고
K/D/R/P가 보정 파일과 일치했다. Jetson에서는 25초간 JPEG 358프레임
(수신 구간 기준 14.68 FPS, 영상 데이터 5.08 Mbps)을 디코딩했으며 오류는 0건이었다.
Jetson에서도 `camera_info` 크기 640×480과 보정 완료 상태를 확인했다.
이는 짧은 수신 시험 결과이며 무선망 부하에 따라 수신률은 달라질 수 있다.

현재 카메라는 도메인 11, 일반 통신 설정으로 `nohup` 실행 중이다.
확인 당시 PID는 2526이며 부팅 자동 실행은 추가하지 않았다. 재실행 전
`pgrep -a -x camera_node`로 중복 여부를 확인한다.

```bash
# Pi에서 실제 사용한 재실행 명령 (이미 실행 중이면 중복 실행하지 않음)
nohup env -u FASTRTPS_DEFAULT_PROFILES_FILE -u FASTDDS_DEFAULT_PROFILES_FILE \
  ROS_DOMAIN_ID=11 bash /home/waffle/camera_ws/scripts/run_waffle_camera.sh \
  > /home/waffle/camera_ws/camera-calibrated-default.log 2>&1 < /dev/null &
```

진단 중 관찰한 공유 메모리 잠금 오류와 UDP 송신 대기는 일반 설정 재시험에서
재현되지 않았다. 보정 창의 원본 구독을 종료하고 재시작한 뒤 정상화됐지만,
구독 종료만을 단독 원인으로 확정한 것은 아니다. 임시 UDP/non-blocking 설정은
실행 스크립트나 전역 환경에 반영하지 않았고, `/tmp/camera-calibration-udp.xml`은
검증 후 삭제했다. 이전 진단 수치는 실시간 검증 JSON의 `previous_diagnostic`에 보존했다.

보정값을 로드해도 `/camera/image_raw` 및 압축 원본의 픽셀은 자동으로 보정되지 않는다.
왜곡 보정 영상이 필요하면 이 CameraInfo를 사용해 `image_proc` 또는 OpenCV로
별도 보정 영상을 생성한다.

## 1. 보정판 확인 (새 판이 필요할 때만 인쇄)

현재 사용자는 기존 보정판을 사용한다. 내부 교차점 수와 실제 한 칸 크기를
그 판에서 확인해야 한다. 전체 크기 259×201mm와 연속 4칸 115mm만으로는
칸 수를 확정하지 않는다. 가로 9칸×세로 7칸이고 두 방향의 4칸이 각각 115mm라면
내부 교차점은 8×6, 한 칸은 0.02875m다. 아래 PDF의 9×6/0.025 옵션과 혼동하지 않는다.

파일: [A4 보정판 PDF](../output/pdf/waffle_checkerboard_9x6_25mm_A4.pdf)

- A4 가로, 실제 크기/100%로 한 장에 인쇄한다.
- 페이지에 맞추기, 축소/확대, 테두리 없는 인쇄의 자동 확대 기능은 끈다.
- 흰색·검은색 칸은 10×7개, **내부 교차점은 9×6개**, 설계상 한 칸은 25mm다.
- 전체 격자는 250×175mm다. 인쇄 후 가로와 세로 모두 연속 4칸을 재어 100mm인지 확인한다.
- 가로·세로 배율이 다르거나 격자가 찌그러졌다면 재인쇄한다.
- 무광 용지에 선명하게 인쇄하고 평평하고 단단한 판에 주름 없이 붙인다.
  격자 주변 흰 여백을 유지한다. 광택 코팅이나 격자 위 테이프는 반사를 만들 수 있다.

인쇄물을 자로 확인하기 전에는 `--square 0.025`가 실제 크기와 맞는다고 가정하지 않는다.
균일하게 다른 크기로 출력됐다면 측정한 한 칸 길이를 미터 단위로 입력한다.
예: 실제 24.5mm이면 `--square 0.0245`.

## 2. 개발 PC에 보정 GUI 설치

2026-10-27 재확인 결과 개발 PC에 camera_calibration 5.0.13이 설치되어 있다.
다음 명령은 설치되지 않은 PC에서만 **일반 터미널**로 실행한다.

```bash
sudo apt update
sudo apt install ros-jazzy-camera-calibration
source /opt/ros/jazzy/setup.bash
ros2 pkg prefix camera_calibration
```

GUI를 보면서 판을 움직일 수 있는 PC에서 실행한다. Pi나 Jetson에 이 GUI를 설치할
필요는 없다. 기본 Python에서 NumPy/OpenCV 호환 오류가 발생하면 시스템 패키지를
삭제하기 전에 아래 조합으로 import가 되는지 확인한다.

```bash
PYTHONPATH=/usr/lib/python3/dist-packages /usr/bin/python3 \
  -c 'import numpy, cv2; print(numpy.__version__, cv2.__version__)'
```

현재 실행 환경에서는 시스템 NumPy 1.26.4 / OpenCV 4.6.0 조합이 동작했다.
`scripts/run_camera_calibration.sh`는 ROS 환경을 읽고 이 시스템 패키지 경로를
우선하도록 설정한 뒤 `/usr/bin/python3`로 보정 프로그램을 실행한다.
NumPy, OpenCV, cv_bridge, 보정 모듈 import 및 스크립트의 `--help` 실행을 검증했다.
전역 Python 패키지나 `.bashrc`를 변경하는 방식은 사용하지 않았다.

## 3. Pi 카메라와 보정 프로그램 실행

Pi에는 640×480, sensor_mode 1640×1232 설정을 사용한다. 보정과 실제 사용 시
해상도, 센서 모드, 렌즈 초점, 회전·크롭 설정을 같게 유지한다.
카메라가 이미 실행 중이면 다시 실행하지 않는다. 2026-10-27 확인 당시에는 실행 중이었다.

```bash
# Waffle Pi 터미널
ROS_DOMAIN_ID=11 bash /home/waffle/camera_ws/scripts/run_waffle_camera.sh
```

기존 보정판이 **내부 교차점 8×6, 한 칸 28.75mm**로 확인됐을 때 개발 PC에서 실행한다.

```bash
ROS_DOMAIN_ID=11 bash /home/sojeong/turtlebot3_ws/scripts/run_camera_calibration.sh \
  --size 8x6 --square 0.02875
```

제공한 PDF를 새로 인쇄해 실제 한 칸 25mm를 확인한 경우에만
`--size 9x6 --square 0.025`를 사용한다. 실행 스크립트에는 판 크기를 고정하지 않았다.

Pi에서 `/camera/set_camera_info` 서비스를 확인했다. 보정용으로 원본 영상 토픽을
구독하므로 이 시간에는 압축 영상만 받을 때보다 네트워크 사용량이 늘어난다.
화면이 심하게 지연되면 오래된 화면으로 계속 촬영하지 말고 연결 상태를 확인한다.

## 4. 판을 움직여 다양한 관측 수집

- 로봇과 카메라는 정지시킨다. 밝고 고른 조명에서 반사와 흔들림을 줄인다.
- 처음에는 판 전체와 흰 여백이 화면 안에 들어오도록 정면으로 보여준다.
- 왼쪽·오른쪽·위·아래·모서리로 위치를 바꾸고, 가까이·멀리 움직인다.
- 판을 좌우·상하로 기울여 다양한 각도를 보여준다. 정면 영상만 모으지 않는다.
- 각 자세에서 잠깐 멈춰 선명한 교차점을 검출할 수 있게 한다.
- GUI의 X, Y, Size, Skew 수집 상태를 고르게 채운 뒤 CALIBRATE를 누른다.

인쇄물을 모니터에 띄워 대신 촬영하면 반사·화소 무늬·실제 크기 확인 문제가
생길 수 있으므로 제공한 PDF를 인쇄해 사용하는 편이 좋다.

## 5. 결과 저장·검증·적용

CALIBRATE 후 SAVE로 관측 영상과 결과를 저장한다. 기본 저장 파일은 개발 PC의
`/tmp/calibrationdata.tar.gz`이므로 이전 파일이 있다면 먼저 별도 보관한다.
SAVE와 COMMIT은 다르다. SAVE는 PC에 자료를 남기고, COMMIT은 Pi의
`set_camera_info` 서비스로 값을 적용한다.

계산 결과의 재투영 오차, 화면 전체의 표본 분포, 새 관측 영상에서의 직선 왜곡을
확인한 후 적용한다. 오차 숫자가 작다는 이유만으로 충분하다고 판단하지 않는다.
검증된 YAML과 촬영 설정을 프로젝트에 보관하고 Pi에서 로드하도록 연결한다.
보정 완료 판정은 `/camera/camera_info`의 실제 영상 크기와 유효한 K/D 행렬,
카메라 재시작 후 같은 보정값 로드까지 확인한 뒤 내린다.

## 준비 작업 기록

- PDF는 ReportLab으로 생성한 벡터 격자이며 A4 297×210mm를 확인했다.
- PDF를 PNG로 렌더링해 육안 확인 후 OpenCV에서 내부 교차점 54개 검출을 확인했다.
- 인쇄 후의 실제 크기·평탄도 검사는 사용자가 별도로 해야 한다.
- 초기 준비 시에는 Pi의 실행 상태와 서비스만 조회했다. 이후 보정 파일 지정과 카메라 재시작을 수행했으며 추가 권한 변경은 없었다.
- PC의 보정 패키지 설치와 모듈 로드를 확인했다. 이후 수집·계산·저장·적용 결과는 문서 상단에 기록했다.

공식 참고:
[ROS 단안 보정 안내](https://github.com/ros-perception/image_pipeline/blob/rolling/camera_calibration/doc/tutorial_mono.rst),
[Nav2 Jazzy 카메라 보정](https://docs.nav2.org/jazzy/tutorials/general_tutorials/camera_calibration/camera_calibration/).
