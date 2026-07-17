#!/bin/bash
# ROS 2 Humble 安装脚本(WSL Ubuntu 22.04,root 执行)
set -x
export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y curl gnupg lsb-release locales software-properties-common
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
add-apt-repository -y universe

# ROS 2 apt 源
curl -sSL --retry 3 https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list
apt-get update

apt-get install -y \
  ros-humble-ros-base \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pytest \
  ros-humble-launch-testing \
  ros-humble-launch-testing-ros \
  ros-humble-ament-cmake-gtest \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-xacro

rosdep init 2>/dev/null || true
rosdep update --include-eol-distros || true

echo "INSTALL_DONE_RC=$?"
source /opt/ros/humble/setup.bash
ros2 --help >/dev/null 2>&1 && echo "ROS2_OK" || echo "ROS2_FAIL"
which colcon && echo "COLCON_OK"
