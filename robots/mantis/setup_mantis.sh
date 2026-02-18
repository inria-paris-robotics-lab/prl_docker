#!/usr/bin/env bash
set -e

WS_DIR="/home/ros/share/mantis_ws"
SRC_DIR="$WS_DIR/src"

if [ ! -d "$SRC_DIR/prl_ur5_ros2" ]; then
  echo "Mantis workspace source missing, setting it up..."

  mkdir -p "$SRC_DIR"
  cd "$SRC_DIR"

  git clone --recursive https://github.com/inria-paris-robotics-lab/prl_ur5_ros2.git

  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  vcs import < "prl_ur5_ros2/dependencies.repos"

  cd "$WS_DIR"
  rosdep update
  rosdep install -r --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" -y
  colcon build --symlink-install
fi

exec "$@"
