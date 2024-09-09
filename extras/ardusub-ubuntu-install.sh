#!/bin/bash

mkdir -p "/opt/ardupilot_dave" 2>/dev/null
if [ $? -ne 0 ]; then
  echo "Insufficient privileges to create directory in /opt."
  sudo mkdir -p "/opt/ardupilot_dave" && cd "/opt/ardupilot_dave" || exit
else
  mkdir -p "/opt/ardupilot_dave" && cd "/opt/ardupilot_dave" || exit
fi

# Really should do version pinning but Sub-4.5 is waaaay behind master
# (e.g. it doesn't know about "noble" yet)
export ARDUPILOT_RELEASE=master
mkdir -p "/opt/ardupilot_dave/ardupilot" 2>/dev/null
if [ $? -ne 0 ]; then
  sudo git clone -b $ARDUPILOT_RELEASE https://github.com/ArduPilot/ardupilot.git --recurse-submodules
else
  git clone -b $ARDUPILOT_RELEASE https://github.com/ArduPilot/ardupilot.git --recurse-submodules
fi

# Install ArduSub dependencies
cd "/opt/ardupilot_dave/ardupilot" || exit
export SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1
# Do not install the STM development tools
export DO_AP_STM_ENV=0
# Do not activate the Ardupilot venv by default
export DO_PYTHON_VENV_ENV=0
Tools/environment_install/install-prereqs-ubuntu.sh -y

# Build ArduSub
cd "/opt/ardupilot_dave/ardupilot" || exit
# needs python binary (e.g. sudo apt install python-is-python3)
mkdir -p "/opt/ardupilot_dave/ardupilot/mktest" 2>/dev/null
if [ $? -ne 0 ]; then
  sudo modules/waf/waf-light configure --board still \
    && sudo modules/waf/waf-light build --target bin/ardusub
else
  modules/waf/waf-light configure --board still \
    && modules/waf/waf-light build --target bin/ardusub
fi

# Clone ardupilot_gazebo code
cd "/opt/ardupilot_dave" || exit
mkdir -p "/opt/ardupilot_dave/ardupilot_gazebo" 2>/dev/null
if [ $? -ne 0 ]; then
  sudo git clone https://github.com/ArduPilot/ardupilot_gazebo.git
else
  git clone https://github.com/ArduPilot/ardupilot_gazebo.git
fi

# Install ardupilot_gazebo plugin
# Check if the directory creation was successful
mkdir -p "/opt/ardupilot_dave/ardupilot_gazebo/build" 2>/dev/null
if [ $? -ne 0 ]; then
  echo "Insufficient privileges to create directory in /opt. Using sudo for cmake and make."
  sudo mkdir -p "/opt/ardupilot_dave/ardupilot_gazebo/build" \
  && cd "/opt/ardupilot_dave/ardupilot_gazebo/build" || exit
  sudo cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && sudo make -j2
else
  cd "/opt/ardupilot_dave/ardupilot_gazebo/build" || exit
  cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j2
fi

# Add results of ArduSub build
export PATH=/opt/ardupilot_dave/ardupilot/build/still/bin:\$PATH
# Optional: add autotest to the PATH, helpful for running sim_vehicle.py
export PATH=/opt/ardupilot_dave/ardupilot/Tools/autotest:\$PATH
# Add ardupilot_gazebo plugin
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_dave/ardupilot_gazebo/build:\$GZ_SIM_SYSTEM_PLUGIN_PATH
# Add ardupilot_gazebo models and worlds
export GZ_SIM_RESOURCE_PATH=/opt/ardupilot_dave/ardupilot_gazebo/models:/opt/ardupilot_dave/ardupilot_gazebo/worlds:\$GZ_SIM_RESOURCE_PATH