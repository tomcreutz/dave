#!/bin/bash
# Script to install development tools and libraries for robotics and simulation

echo "Updating package lists..."
sudo apt update

echo "Performing full system upgrade (this might take a while)..."
sudo apt full-upgrade -y

echo "Installing essential tools and libraries..."
sudo apt install -y \
    build-essential \
    cmake \
    cppcheck \
    curl \
    git \
    gnupg \
    libeigen3-dev \
    libgles2-mesa-dev \
    lsb-release \
    pkg-config \
    protobuf-compiler \
    python3-dbg \
    python3-pip \
    python3-venv \
    qtbase5-dev \
    ruby \
    software-properties-common \
    sudo \
    wget

echo "Installation complete!"

echo "Installing Signing Keys for ROS and Gazebo..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo wget https://packages.osrfoundation.org/gazebo.gpg \
    -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

ARCH=$(dpkg --print-architecture)
# shellcheck disable=SC1091
UBUNTU_CODENAME=$( . /etc/os-release && echo "$UBUNTU_CODENAME")
REPO="deb [arch=$ARCH signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2-testing/ubuntu $UBUNTU_CODENAME main"
echo "$REPO" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

DISTRO=$(lsb_release -cs)
REPO="deb [arch=$ARCH signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $DISTRO main"
echo "$REPO" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null

echo "Installing ROS Gazebo framework..."
sudo apt update && apt install -y \
    python3-rosdep \
    python3-rosinstall-generator \
    python3-vcstool \
    gz-harmonic \
    ros-jazzy-desktop-full \
    ros-jazzy-gz-plugin-vendor \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-effort-controllers \
    ros-jazzy-geographic-info \
    ros-jazzy-image-view \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joy \
    ros-jazzy-joy-teleop \
    ros-jazzy-key-teleop \
    ros-jazzy-moveit-planners \
    ros-jazzy-moveit-simple-controller-manager \
    ros-jazzy-moveit-ros-visualization \
    ros-jazzy-pcl-ros \
    ros-jazzy-robot-localization \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-ros-base \
    ros-jazzy-ros2-controllers \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-rviz2 \
    ros-jazzy-teleop-tools \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-tf2-tools \
    ros-jazzy-urdfdom-py \
    ros-jazzy-xacro \
    ros-dev-tools \
    ros-jazzy-ros-gz \
