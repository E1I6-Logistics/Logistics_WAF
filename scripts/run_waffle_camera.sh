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
# Launch must own the actual camera process so shutdown escalation reaches it.
# `ros2 run` adds a wrapper whose termination can leave camera_node orphaned.
camera_prefix="$(ros2 pkg prefix camera_ros)"
exec "$camera_prefix/lib/camera_ros/camera_node" --ros-args \
  --params-file "$project_root/config/waffle_camera.yaml" "$@"
