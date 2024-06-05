FROM arm64v8/ubuntu:24.04
ARG BRANCH="ros2"
ARG ROS_DISTRO="jazzy"

# update and upgrade libs
RUN apt-get update \
    && apt-get -y upgrade \
    && rm -rf /tmp/*

# Install basics
ENV DEBIAN_FRONTEND noninteractive
ENV DEBCONF_NONINTERACTIVE_SEEN=true
# hadolint ignore=DL3008
RUN apt-get install -y --no-install-recommends \
    sudo tzdata build-essential gfortran automake \
    bison flex libtool git wget locales \
    software-properties-common

# Locale for UTF-8
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# Install Utilities
# hadolint ignore=DL3008
RUN apt-get -y install --no-install-recommends \
    x11-apps mesa-utils xauth \
    && rm -rf /tmp/*

ADD https://raw.githubusercontent.com/IOES-Lab/dave/$BRANCH/\
extras/ros-jazzy-gz-harmonic-install.sh install.sh
RUN bash install.sh

ENV ROS_UNDERLAY /root/ws_dave/install
WORKDIR $ROS_UNDERLAY/../src

ADD https://raw.githubusercontent.com/IOES-Lab/dave/$BRANCH/\
extras/repos/dave.jazzy.repos dave.repos
RUN vcs import < dave.repos

RUN apt-get update && rosdep update && \
    rosdep install -iy --from-paths . && \
    rm -rf /var/lib/apt/lists/

WORKDIR $ROS_UNDERLAY/..
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build

# source entrypoint setup
RUN sed --in-place --expression \
    '$i source "$ROS_UNDERLAY/setup.bash"' /ros_entrypoint.sh
