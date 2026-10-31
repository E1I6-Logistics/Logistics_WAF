#!/usr/bin/env bash
# Run on the Raspberry Pi after building ~/camera_ws.
set -e
camera_workspace="${CAMERA_WS:-$HOME/camera_ws}"
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [ ! -f "$camera_workspace/install/setup.bash" ]; then
  printf 'Camera workspace is not built: %s\n' "$camera_workspace" >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$camera_workspace/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-11}"
exec ros2 run camera_ros camera_node --ros-args \
  --params-file "$project_root/config/waffle_camera.yaml" "$@"
