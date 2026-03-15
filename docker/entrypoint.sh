#!/usr/bin/env bash
set -euo pipefail

WS_ROOT=/ros2_ws

source_setup() {
  set +u
  source "$1"
  set -u
}

source_setup "/opt/ros/${ROS_DISTRO}/setup.bash"

cd "${WS_ROOT}"

echo "[entrypoint] Building workspace in ${WS_ROOT}"
colcon build --packages-select px4_manipulation manipulation_msgs

source_setup "${WS_ROOT}/install/setup.bash"

exec "$@"
