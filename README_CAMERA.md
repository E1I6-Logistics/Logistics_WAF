# Waffle Raspberry Pi 카메라

Raspberry Pi 4의 CSI 카메라(IMX219)를 Pi에서 읽고 Jetson으로 JPEG 영상을
전송하는 구성이다. Ubuntu 24.04 / ROS 2 Jazzy에서 Raspberry Pi의 libcamera
포크와 `camera_ros`를 `/home/waffle/camera_ws`에 함께 빌드한다.
기존 TurtleBot 작업 공간과 시스템 libcamera는 덮어쓰지 않는다.

**검증 완료(2026-10-27):** Pi에서 libcamera와 camera_ros 빌드 성공,
IMX219 10프레임 캡처, 원본 ROS 영상 수신, Jetson의 JPEG 수신·디코딩을 확인했다.
ROS_DOMAIN_ID=11로 25초간 시험해 340프레임을 받았다. 실제 프레임 수신 구간 기준
15.21 FPS, JPEG 평균 44,553바이트, 영상 데이터 약 5.42 Mbps이며 디코딩 오류는 0건이었다.
전송량은 장면에 따라 달라지고 네트워크 프로토콜 부하는 포함하지 않는다.
위 초기 시험 후 노드를 종료했다. 이후 2026-10-27 보정값을 적용하고 카메라를
다시 실행했다. 보정 창 종료 후 일반 통신 설정에서 Pi 원본 14.61 FPS,
Jetson JPEG 14.68 FPS(25초간 358프레임, 디코딩 오류 0)를 확인했다.
현재 카메라는 실행 중이며 상세 상태는 [보정 결과](calibration/README.md)를 참고한다.

## 실행

Waffle에 접속한 뒤 실행한다. 이미 카메라 노드가 실행 중이면 중복 실행하지 않는다.

```bash
ssh waffle@192.168.0.105
ROS_DOMAIN_ID=11 bash /home/waffle/camera_ws/scripts/run_waffle_camera.sh
```

`Ctrl+C`로 종료한다. 부팅 시 자동 실행은 설정하지 않았다.

설정 원본은 `config/waffle_camera.yaml`, 실행 스크립트는
`scripts/run_waffle_camera.sh`이다. Pi에는 각각 `camera_ws/config`,
`camera_ws/scripts`로 복사한다. PC에서 수정했다면 다시 복사해야 반영된다.

```bash
scp config/waffle_camera.yaml waffle@192.168.0.105:/home/waffle/camera_ws/config/
scp calibration/imx219_640x480_20261027.yaml waffle@192.168.0.105:/home/waffle/camera_ws/config/
scp scripts/run_waffle_camera.sh waffle@192.168.0.105:/home/waffle/camera_ws/scripts/
```

초기 설정은 640×480, 약 15 FPS, JPEG 품질 80이다.

| 토픽 | 메시지 | 용도 |
| --- | --- | --- |
| `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | Jetson에서 JPEG 수신 |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | 원본 컬러 영상 (`bgr8`) |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 영상 크기와 보정 정보 |

Jetson에서는 압축 토픽을 구독하고 OpenCV `imdecode` 또는
`cv_bridge.compressed_imgmsg_to_cv2`로 풀어서 처리한다. 이 `camera_ros` 버전은
JPEG 압축을 자체 제공하므로 별도 republish 노드가 필요 없다.
원본 토픽을 원격에서 구독하면 원본 영상도 네트워크로 전달된다.

Jetson의 새 터미널에서는 도메인을 명시한다.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=11
ros2 topic hz /camera/image_raw/compressed
ros2 topic echo /camera/camera_info --once
```

2026-10-27 카메라 내부 보정값을 적용했다. `frame_id`는
`camera_rgb_optical_frame`을 사용한다. 2026-10-29 실물 센서가 IMX219인데
실행 URDF가 R200용임을 확인하여, Waffle URDF의 카메라 부분만 Waffle Pi
기본 구성으로 교체했다. 현재 광학 원점은 base_link 기준 `[0.076, 0, 0.093]m`다.
실행 중 robot_description과 실제 TF를 모두 확인했고 로봇의 기존 LiDAR yaw
3.22rad를 보존했다. 기본 브래킷 위치를 전제로 한 모델 수정이며, 장착 오차의
실측 검증이나 `body_front_x` 측정을 대신하지 않는다.
상세 적용·백업·캐시 상태는 [수정 기록](output/imx219_tf_update/README.md)을 참고한다.
`camera_info_url`은 Pi의 `config/imx219_640x480_20261027.yaml`을 가리킨다.
재시작 후 `camera_info`가 640×480 크기와 저장된 K/D/R/P 행렬을 발행함을 확인했다.
초기 설치 시 발생하던 보정 파일 없음 오류는 새 로그에서 발생하지 않았다.
보정 창 종료 후 일반 통신 설정으로 재시작해 영상 전송이 정상화됨을 확인했다.
임시 UDP 설정은 최종 실행에 사용하지 않는다.
인쇄용 체커보드와 촬영 절차는 [카메라 보정 준비](calibration/README.md)를 참고한다.

## 실제 변경 위치와 확인 명령

| 장비 | 위치 | 변경 내용 |
| --- | --- | --- |
| Waffle Pi | 시스템 apt 패키지 | 아래 빌드 도구·개발 라이브러리와 rosdep 의존성 설치 |
| Waffle Pi | `waffle` 계정 | `sudo -n usermod -aG video waffle`로 카메라 접근 그룹 추가 |
| Waffle Pi | `/home/waffle/camera_ws/src` | libcamera, camera_ros 소스 다운로드 및 호환 패치 적용 |
| Waffle Pi | `/home/waffle/camera_ws/build`, `install`, `log` | colcon이 생성한 빌드 결과·설치 파일·로그 |
| Waffle Pi | `/home/waffle/camera_ws/config/waffle_camera.yaml` | 640×480, 15 FPS, JPEG 품질 80 설정 |
| Waffle Pi | `/home/waffle/camera_ws/scripts/run_waffle_camera.sh` | 환경 설정 후 카메라 실행, 실행 권한 0755 |
| Waffle Pi | `/home/waffle/camera_ws/scripts/build_camera.sh` | 아래 두 패키지 빌드 명령을 순서대로 실행 |
| Waffle Pi | `/home/waffle/camera_ws/camera_ros-libcamera-0.7.patch` | camera_ros 소스 7개 파일의 C++20 API 호환 변경 기록 |
| 개발 PC | `/home/sojeong/turtlebot3_ws/patches/camera_ros-libcamera-0.7.patch` | Pi에서 scp로 가져온 동일 패치 |
| Jetson | `/tmp/verify_waffle_camera.py` | 25초간 압축 영상 수신·디코딩하는 임시 검증 코드 |

`video` 그룹 추가는 새 로그인부터 적용된다. `id`에서 `44(video)`를 확인했다.
시스템 라이브러리를 덮어쓰거나 `.bashrc`, 부팅 자동 실행 설정을 변경하지 않았다.
Jetson의 로그인 기본 도메인은 15이므로 수신할 터미널에서 11을 명시해야 한다.

Pi에서 빌드를 SSH 연결과 독립적으로 실행하고 로그를 확인한 명령:

```bash
cd /home/waffle/camera_ws
nohup bash scripts/build_camera.sh > build-camera.log 2>&1 < /dev/null &
tail -n 30 build-camera.log
```

`nohup`은 SSH 종료 시에도 작업이 계속되게 하고, `&`는 백그라운드 실행,
`> ... 2>&1`은 일반 출력과 오류를 같은 로그에 저장한다.
최종 로그의 `BUILD_EXIT=0`은 두 빌드가 모두 성공했음을 뜻한다.

Pi에서 실제로 수행한 캡처·라이브러리 확인 명령:

```bash
source /opt/ros/jazzy/setup.bash
source /home/waffle/camera_ws/install/setup.bash
cam -l
timeout 20s cam -c 1 --capture=10
ldd /home/waffle/camera_ws/install/camera_ros/lib/libcamera_component.so
```

`cam`은 파일 저장 옵션 없이 10프레임을 캡처한다. `ldd`에서 libcamera가
`/home/waffle/camera_ws/install/libcamera/lib` 아래에서 로드됨을 확인했다.
ROS 실행 로그는 Pi의 `/home/waffle/camera_ws/camera-validation.log`에 남겼다.

## 빌드 기록과 재구성

사용 소스:

- [Raspberry Pi libcamera](https://github.com/raspberrypi/libcamera):
  `c682979b50695b93526adb1e66db33743484b63b` (0.7.2 기반)
- [camera_ros](https://github.com/christianrauch/camera_ros):
  `73bc59749f453cd2ddc5dc25f49a1dedf4415f13` (0.7.0)

아래 명령은 Pi의 새 작업 공간을 준비할 때 사용한다. libcamera와 camera_ros는
같은 작업 공간에서 빌드해 ABI가 일치하도록 한다.

```bash
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  git build-essential python3-rosdep python3-colcon-common-extensions \
  python3-colcon-meson meson ninja-build pkg-config libyaml-dev \
  python3-yaml python3-ply python3-jinja2 libgnutls28-dev openssl \
  libudev-dev libevent-dev libboost-dev libtiff-dev v4l-utils

# 카메라 장치 접근 권한: 변경 후 SSH를 새로 접속한다.
sudo usermod -aG video "$USER"

source /opt/ros/jazzy/setup.bash
mkdir -p ~/camera_ws/src
cd ~/camera_ws
git clone https://github.com/raspberrypi/libcamera.git src/libcamera
git -C src/libcamera checkout c682979b50695b93526adb1e66db33743484b63b
git clone https://github.com/christianrauch/camera_ros.git src/camera_ros
git -C src/camera_ros checkout 73bc59749f453cd2ddc5dc25f49a1dedf4415f13

# PC 프로젝트의 patches/camera_ros-libcamera-0.7.patch를 Pi의 ~/camera_ws로 복사한 뒤 적용한다.
git -C src/camera_ros apply --check /home/waffle/camera_ws/camera_ros-libcamera-0.7.patch
git -C src/camera_ros apply /home/waffle/camera_ws/camera_ros-libcamera-0.7.patch

# rosdep이 초기화되지 않은 새 OS에서는 먼저 sudo rosdep init을 실행한다.
rosdep update --rosdistro jazzy
rosdep install -y --from-paths src/camera_ros --ignore-src \
  --rosdistro jazzy --skip-keys=libcamera \
  --dependency-types=build --dependency-types=buildtool --dependency-types=exec

MAKEFLAGS='-j2' colcon build --packages-select libcamera --executor sequential \
  --meson-args -Dpipelines=rpi/vc4 -Dipas=rpi/vc4 \
  -Dcam=enabled -Dqcam=disabled -Dgstreamer=disabled -Dpycamera=disabled \
  -Dtest=false -Ddocumentation=disabled -Drpi-awb-nn=disabled

source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS='-j2' \
  colcon build --packages-select camera_ros --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DCMAKE_CXX_FLAGS=
source install/setup.bash
cam -l
```

위 패치를 Pi로 복사하는 명령은 **개발 PC**에서 실행한다.

```bash
scp /home/sojeong/turtlebot3_ws/patches/camera_ros-libcamera-0.7.patch \
  waffle@192.168.0.105:/home/waffle/camera_ws/
```

Pi 4에 맞춰 `rpi/vc4`만 빌드하고 빌드 작업 수를 2개로 제한한다.
libcamera 0.7은 C++20의 `std::span`과 `std::dynamic_extent`를 사용한다.
위 패치는 camera_ros 0.7.0의 이전 API 이름과 헤더를 변경한다.
기존 `-Werror` 검사는 유지하며, `-DCMAKE_CXX_FLAGS=`는 이전 빌드에서
지정한 경고 예외 옵션을 비운다. 패치는 명시한 소스 버전용이며 중복 적용하지 않는다.
`ros-jazzy-camera-ros` 및 일반 ROS libcamera 바이너리를 추가로 설치하면
서로 다른 라이브러리를 로드할 수 있으므로 실행 시 위 작업 공간을 source한다.
