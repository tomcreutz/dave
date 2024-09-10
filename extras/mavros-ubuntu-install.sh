#!/bin/bash

# Source Ros and Gazebo
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash

mkdir -p "/opt/mavros/" && cd "/opt/mavros/" || exit
wget https://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.52.tar.gz && \
    tar xfpz GeographicLib-1.52.tar.gz && rm GeographicLib-1.52.tar.gz && mv GeographicLib-1.52 GeographicLib
mkdir -p GeographicLib/BUILD && cd GeographicLib/BUILD || exit
../configure && make -j2 && make install
geographiclib-get-geoids egm96-5
geographiclib-get-gravity egm96
geographiclib-get-magnetic emm2015
ln -s /usr/local/share/GeographicLib/geoids/egm96-5.pgm /usr/local/share/GeographicLib/egm96-5.pgm
ln -s /usr/local/share/GeographicLib/gravity/egm96.egm /usr/local/share/GeographicLib/egm96.egm
ln -s /usr/local/share/GeographicLib/magnetic/emm2015.wmm /usr/local/share/GeographicLib/emm2015.wmm
export GEOGRAPHICLIB_GEOID_PATH=/usr/local/share/GeographicLib

# Manually install MAVROS from source
export MAVROS_RELEASE=ros2
export MAVLINK_RELEASE=release/rolling/mavlink
mkdir -p "/opt/mavros/src" && cd "/opt/mavros" || exit
vcs import --force --shallow --retry 0 \
        --input https://raw.githubusercontent.com/IOES-Lab/dave/ardusub_install/extras/repos/mavros.jazzy.repos src

# Install MAVROS dependencies
apt update && apt install libasio-dev libtinyxml2-dev -y

# Build
MAKEFLAGS="-j2" ROS_PYTHON_VERSION=3 colcon build --cmake-args -DCMAKE_MODULE_PATH=/usr/local/share/cmake/GeographicLib:\$CMAKE_MODULE_PATH -Wno-dev -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release

# Source mavros
# shellcheck disable=SC1091
source /opt/mavros/install/setup.bash