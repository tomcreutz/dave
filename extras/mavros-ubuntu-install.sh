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






# # Add results of ArduSub build
# export PATH=/opt/ardupilot_dave/ardupilot/build/still/bin:\$PATH
# # Optional: add autotest to the PATH, helpful for running sim_vehicle.py
# export PATH=/opt/ardupilot_dave/ardupilot/Tools/autotest:\$PATH
# # Add ardupilot_gazebo plugin
# export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_dave/ardupilot_gazebo/build:\$GZ_SIM_SYSTEM_PLUGIN_PATH
# # Add ardupilot_gazebo models and worlds
# export GZ_SIM_RESOURCE_PATH=/opt/ardupilot_dave/ardupilot_gazebo/models:/opt/ardupilot_dave/ardupilot_gazebo/worlds:\$GZ_SIM_RESOURCE_PATH