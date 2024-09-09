#!/bin/bash

mkdir -p "/opt/ardupilot_dave" && cd "/opt/ardupilot_dave" || exit
# Really should do version pinning but Sub-4.5 is waaaay behind master
# (e.g. it doesn't know about "noble" yet)
export ARDUPILOT_RELEASE=master
git clone -b $ARDUPILOT_RELEASE https://github.com/ArduPilot/ardupilot.git --recurse-submodules

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
apt-get install -y python-is-python3 python3-future
modules/waf/waf-light configure --board still \
    && modules/waf/waf-light build --target bin/ardusub

# Clone ardupilot_gazebo code
cd "/opt/ardupilot_dave" || exit
git clone https://github.com/ArduPilot/ardupilot_gazebo.git

# Install ardupilot_gazebo plugin
mkdir -p "/opt/ardupilot_dave/ardupilot_gazebo/build" \
&& cd "/opt/ardupilot_dave/ardupilot_gazebo/build" || exit
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j2

# Add results of ArduSub build
export PATH=/opt/ardupilot_dave/ardupilot/build/still/bin:\$PATH
# Optional: add autotest to the PATH, helpful for running sim_vehicle.py
export PATH=/opt/ardupilot_dave/ardupilot/Tools/autotest:\$PATH
# Add ardupilot_gazebo plugin
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_dave/ardupilot_gazebo/build:\$GZ_SIM_SYSTEM_PLUGIN_PATH
# Add ardupilot_gazebo models and worlds
export GZ_SIM_RESOURCE_PATH=/opt/ardupilot_dave/ardupilot_gazebo/models:/opt/ardupilot_dave/ardupilot_gazebo/worlds:\$GZ_SIM_RESOURCE_PATH