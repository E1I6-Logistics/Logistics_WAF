#!/usr/bin/env bash
# Run on the development PC; use Ubuntu's matching NumPy/OpenCV packages.
set -e

if [ "$#" -eq 0 ]; then
  printf 'Usage: bash %s --size COLSxROWS --square METERS [calibrator options]\n' "$0" >&2
  printf 'COLSxROWS counts inner corners; METERS is the measured square edge.\n' >&2
  exit 2
fi

source /opt/ros/jazzy/setup.bash
calibrator=/opt/ros/jazzy/lib/camera_calibration/cameracalibrator
if [ ! -f "$calibrator" ]; then
  printf 'Install on the PC: sudo apt install ros-jazzy-camera-calibration\n' >&2
  exit 1
fi

# Keep /usr/local NumPy 2.x from shadowing the NumPy used by apt-built cv2.
export PYTHONPATH="/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-11}"

exec /usr/bin/python3 "$calibrator" "$@" \
  --ros-args -r image:=/camera/image_raw \
  -r camera/set_camera_info:=/camera/set_camera_info
