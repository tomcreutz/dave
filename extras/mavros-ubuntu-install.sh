#!/bin/bash

mkdir -p "/opt/mavros" 2>/dev/null
if [ $? -ne 0 ]; then
  echo "Insufficient privileges to create directory in /opt."
  sudo mkdir -p "/opt/mavros" && cd "/opt/mavros" || exit
  sudo wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && sudo ./install_geographiclib_datasets.sh
else
  cd "/opt/mavros" || exit
  wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && sudo ./install_geographiclib_datasets.sh
fi

# Manually install MAVROS from source
export MAVROS_RELEASE=ros2
export MAVLINK_RELEASE=release/rolling/mavlink
mkdir -p "/opt/mavros/src" 2>/dev/null
if [ $? -ne 0 ]; then
  echo "Insufficient privileges to create directory in /opt."
  sudo mkdir -p "/opt/mavros/src" && cd "/opt/mavros/src" || exit
  sudo git clone --depth 1 -b $MAVROS_RELEASE https://github.com/mavlink/mavros.git
  sudo git clone --depth 1 --recursive -b $MAVLINK_RELEASE https://github.com/mavlink/mavlink-gbp-release.git mavlink
  # - mavgen uses future.standard_library for backwards compatibility with Python2;
  #   However, this caused issues with Python 3.12 installed in "noble".
  #   Comment those lines out in mavlink.
  #
  # - Fix linkage for yaml-cpp in mavros_extra_plugins
  sudo sed -i -e 's/^from future import standard_library/#from future import standard_library/' \
  -e 's/standard_library.install_aliases()/#standard_library.install_aliases()/' \
  mavlink/pymavlink/generator/mavgen.py && \
  sudo sed -i -e 's/^# find_package(yaml_cpp REQUIRED)/find_package(yaml-cpp REQUIRED)/' \
  -e '/^ament_target_dependencies(mavros_extras_plugins$/i target_link_libraries(mavros_extras_plugins yaml-cpp::yaml-cpp)' \
  -e '/^ament_target_dependencies(mavros_extras$/i target_link_libraries(mavros_extras yaml-cpp::yaml-cpp)' \
  mavros/mavros_extras/CMakeLists.txt
else
  cd "/opt/mavros/src" || exit
  git clone --depth 1 -b $MAVROS_RELEASE https://github.com/mavlink/mavros.git
  git clone --depth 1 --recursive -b $MAVLINK_RELEASE https://github.com/mavlink/mavlink-gbp-release.git mavlink
  # - mavgen uses future.standard_library for backwards compatibility with Python2;
  #   However, this caused issues with Python 3.12 installed in "noble".
  #   Comment those lines out in mavlink.
  #
  # - Fix linkage for yaml-cpp in mavros_extra_plugins
  sed -i -e 's/^from future import standard_library/#from future import standard_library/' \
  -e 's/standard_library.install_aliases()/#standard_library.install_aliases()/' \
  mavlink/pymavlink/generator/mavgen.py && \
  sed -i -e 's/^# find_package(yaml_cpp REQUIRED)/find_package(yaml-cpp REQUIRED)/' \
  -e '/^ament_target_dependencies(mavros_extras_plugins$/i target_link_libraries(mavros_extras_plugins yaml-cpp::yaml-cpp)' \
  -e '/^ament_target_dependencies(mavros_extras$/i target_link_libraries(mavros_extras yaml-cpp::yaml-cpp)' \
  mavros/mavros_extras/CMakeLists.txt
fi
# Build
cd "/opt/mavros" || exit
MAKEFLAGS="-j2" colcon build --cmake-args -DBUILD_TESTING=OFF


# Source mavros
# shellcheck disable=SC1091
source /opt/mavros/install/setup.bash