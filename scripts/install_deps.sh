#!/usr/bin/env bash
# SwarmX system dependency installer (Ubuntu 24.04 "noble").
# Installs: ROS 2 Jazzy desktop, Gazebo Harmonic (via ros_gz), Nav2, slam_toolbox,
#           rmw_zenoh, colcon/rosdep tooling and the build deps for ARGoS3.
#
# Usage:   sudo bash scripts/install_deps.sh
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "Run with sudo:  sudo bash $0" >&2
  exit 1
fi

. /etc/os-release
if [[ "${VERSION_CODENAME}" != "noble" ]]; then
  echo "This script targets Ubuntu 24.04 (noble); found ${VERSION_CODENAME}." >&2
  exit 1
fi

export DEBIAN_FRONTEND=noninteractive

echo "==> Locale + base tools"
apt-get update
apt-get install -y locales software-properties-common curl gnupg lsb-release git build-essential cmake
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
add-apt-repository -y universe

echo "==> ROS 2 apt source"
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
  dpkg -i /tmp/ros2-apt-source.deb
fi
apt-get update

echo "==> ROS 2 Jazzy + Gazebo Harmonic + Nav2 + SLAM + Zenoh"
apt-get install -y \
  ros-jazzy-desktop \
  ros-dev-tools \
  ros-jazzy-ros-gz \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-rmw-zenoh-cpp \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-tf-transformations \
  python3-numpy python3-pytest python3-yaml python3-transforms3d

echo "==> ARGoS3 build dependencies (ARGoS itself is built without sudo by scripts/build_argos.sh)"
apt-get install -y \
  libfreeimage-dev libfreeimageplus-dev \
  qtbase5-dev libqt5opengl5-dev freeglut3-dev libxi-dev libxmu-dev \
  liblua5.3-dev lua5.3 libgraphviz-dev graphviz

echo "==> rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi

echo
echo "Done. Next (as your normal user, no sudo):"
echo "  rosdep update"
echo "  bash scripts/build_ws.sh"
