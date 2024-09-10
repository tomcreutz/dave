#!/bin/bash

mkdir -p "/opt/mavros/GeographicLib" 2>/dev/null
if [ $? -ne 0 ]; then
  echo "Insufficient privileges to create directory in /opt."
  sudo mkdir -p "/opt/mavros/" && cd "/opt/mavros/" || exit
  sudo wget https://github.com/ObjSal/GeographicLib/archive/refs/tags/v1.44.tar.gz && \
      sudo tar xfpz v1.44.tar.gz && sudo rm v1.44.tar.gz && sudo mv GeographicLib-1.44 GeographicLib
  sudo mkdir -p GeographicLib/BUILD && cd GeographicLib/BUILD || exit
  sudo ../configure && sudo make -j2 && sudo make install
  sudo geographiclib-get-geoids egm96-5
  sudo geographiclib-get-gravity egm96
  sudo geographiclib-get-magnetic emm2015
else
  cd "/opt/mavros/" || exit
  wget https://github.com/ObjSal/GeographicLib/archive/refs/tags/v1.44.tar.gz && \
      tar xfpz v1.44.tar.gz && rm v1.44.tar.gz && mv GeographicLib-1.44 GeographicLib
  mkdir -p GeographicLib/BUILD && cd GeographicLib/BUILD || exit
  ../configure && make -j2 && make install
  geographiclib-get-geoids egm96-5
  geographiclib-get-gravity egm96
  geographiclib-get-magnetic emm2015
fi
export GEOGRAPHICLIB_GEOID_PATH=/usr/local/share/GeographicLib

# Manually install MAVROS from source
export MAVROS_RELEASE=ros2
export MAVLINK_RELEASE=release/rolling/mavlink
mkdir -p "/opt/mavros/src" 2>/dev/null
if [ $? -ne 0 ]; then
  echo "Insufficient privileges to create directory in /opt."
  sudo mkdir -p "/opt/mavros/src" && cd "/opt/mavros" || exit
  sudo vcs import --force --shallow --retry 0 \
          --input https://raw.githubusercontent.com/IOES-Lab/dave/ardusub_install/extras/repos/mavros.jazzy.repos src
  # Build
  MAKEFLAGS="-j2" ROS_PYTHON_VERSION=3 sudo colcon build --cmake-args -DCMAKE_MODULE_PATH=/usr/local/share/cmake/GeographicLib:\$CMAKE_MODULE_PATH -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
else
  vcs import --force --shallow --retry 0 \
          --input https://raw.githubusercontent.com/IOES-Lab/dave/ardusub_install/extras/repos/mavros.jazzy.repos src
  # Build
  MAKEFLAGS="-j2" ROS_PYTHON_VERSION=3 colcon build --cmake-args -DCMAKE_MODULE_PATH=/usr/local/share/cmake/GeographicLib:\$CMAKE_MODULE_PATH -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
fi

# Source mavros
# shellcheck disable=SC1091
source /opt/mavros/install/setup.bash