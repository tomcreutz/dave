#!/bin/bash

mkdir -p "$HOME/ardupilot_dave" && cd "$HOME/ardupilot_dave" || exit
# Really should do version pinning but Sub-4.5 is waaaay behind master
# (e.g. it doesn't know about "noble" yet)
export ARDUPILOT_RELEASE=master
git clone -b $ARDUPILOT_RELEASE https://github.com/ArduPilot/ardupilot.git --recurse-submodules

# Install ArduSub dependencies
cd "$HOME/ardupilot_dave/ardupilot" || exit
export SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1
# Do not install the STM development tools
export DO_AP_STM_ENV=0
# Do not activate the Ardupilot venv by default
export DO_PYTHON_VENV_ENV=0
Tools/environment_install/install-prereqs-ubuntu.sh -y

# Build ArduSub
cd "$HOME/ardupilot_dave/ardupilot" || exit
modules/waf/waf-light configure --board still \
    && modules/waf/waf-light build --target bin/ardusub

# Clone ardupilot_gazebo code
cd "$HOME/ardupilot_dave" || exit
git clone https://github.com/ArduPilot/ardupilot_gazebo.git

# Install ardupilot_gazebo plugin
mkdir -p "$HOME/ardupilot_dave/ardupilot_gazebo/build" && cd "$HOME/ardupilot_dave/ardupilot_gazebo/build" || exit
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j4