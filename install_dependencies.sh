#!/usr/bin/env bash
set -eo pipefail

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble." >&2
  exit 1
fi

sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-numpy \
  python3-scipy \
  ros-humble-joy \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers

source /opt/ros/humble/setup.bash
if ! rosdep db >/dev/null 2>&1; then
  sudo rosdep init || true
  rosdep update
fi
rosdep install --from-paths "$(dirname "$0")/src" --ignore-src -r -y
