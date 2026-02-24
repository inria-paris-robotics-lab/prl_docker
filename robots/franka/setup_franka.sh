#!/usr/bin/env bash
set -e

WS_DIR="/home/ros/share/franka_ws"
SRC_DIR="$WS_DIR/src"

if [ ! -d "$SRC_DIR/prl_franka" ]; then
  echo "PRL Franka workspace source missing, setting it up..."

  mkdir -p "$SRC_DIR"
  cd "$SRC_DIR"

  git clone --recursive https://github.com/inria-paris-robotics-lab/prl_franka.git
fi
if [ ! -d "$WS_DIR/install" ]; then
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  cd "$WS_DIR"
  rosdep update
  rosdep install -r --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" -y --skip-keys "warehouse_ros_mongo"
  colcon build --symlink-install
fi
exec "$@"
