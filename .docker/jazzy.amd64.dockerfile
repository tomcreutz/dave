ARG ROS_DISTRO="jazzy"
FROM osrf/ros:$ROS_DISTRO-desktop-full
ARG BRANCH="ros2"

# Install Utilities
# hadolint ignore=DL3008
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    sudo xterm init systemd snapd vim net-tools \
    curl wget git build-essential cmake cppcheck \
    gnupg libeigen3-dev libgles2-mesa-dev \
    lsb-release pkg-config protobuf-compiler \
    python3-dbg python3-pip python3-venv python3-pexpect \
    python-is-python3 python3-future python3-wxgtk4.0 \
    qtbase5-dev ruby dirmngr gnupg2 nano xauth \
    software-properties-common htop libtool \
    x11-apps mesa-utils bison flex automake \
    && rm -rf /var/lib/apt/lists/

# Prereqs for Ardupilot - Ardusub
ADD --chown=root:root --chmod=0644 https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list /etc/ros/rosdep/sources.list.d/00-gazebo.list
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" |  tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
    libgz-sim8-dev rapidjson-dev libopencv-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    && rm -rf /var/lib/apt/lists/

# Locale for UTF-8
RUN truncate -s0 /tmp/preseed.cfg && \
   (echo "tzdata tzdata/Areas select Etc" >> /tmp/preseed.cfg) && \
   (echo "tzdata tzdata/Zones/Etc select UTC" >> /tmp/preseed.cfg) && \
   debconf-set-selections /tmp/preseed.cfg && \
   rm -f /etc/timezone && \
   dpkg-reconfigure -f noninteractive tzdata
# hadolint ignore=DL3008
RUN apt-get update && \
    apt-get -y install --no-install-recommends locales tzdata \
    && rm -rf /tmp/*
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# Make user (assume host user has 1000:1000 permission)
ARG USER=docker
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN adduser --shell /bin/bash --disabled-password --gecos '' $USER \
    && echo "$USER:$USER" | chpasswd && adduser $USER sudo \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Install ROS-Gazebo framework
ADD https://raw.githubusercontent.com/IOES-Lab/dave/$BRANCH/\
extras/ros-jazzy-binary-gz-harmonic-source-install.sh install.sh
RUN bash install.sh

# Install Ardupilot - Ardusub
ADD https://raw.githubusercontent.com/IOES-Lab/dave/dockertest/\
extras/ardusub-ubuntu-install.sh install.sh
RUN bash install.sh
# Install mavros
ADD https://raw.githubusercontent.com/IOES-Lab/dave/dockertest/\
extras/mavros-ubuntu-install.sh install.sh
RUN bash install.sh

# Set up Dave workspace
ENV DAVE_WS=/opt/dave_ws
WORKDIR $DAVE_WS/src

ADD https://raw.githubusercontent.com/IOES-Lab/dave/$BRANCH/\
extras/repos/dave.$ROS_DISTRO.repos $DAVE_WS/dave.repos
RUN vcs import --shallow --input $DAVE_WS/dave.repos

# Install dave dependencies
RUN apt-get update && rosdep update && \
    rosdep install -iy --from-paths . && \
    rm -rf /var/lib/apt/lists/

# Compile Dave
WORKDIR $DAVE_WS
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build

# source entrypoint setup
RUN touch /ros_entrypoint.sh && sed --in-place --expression \
    '$i source "/opt/dave_ws/install/setup.bash"' /ros_entrypoint.sh

# Source ROS and Gazebo
RUN sed --in-place --expression \
'$i source "/opt/ros/jazzy/setup.bash"' /ros_entrypoint.sh && \
sed --in-place --expression \
'$i source "/opt/gazebo/install/setup.bash"' /ros_entrypoint.sh && \
sed --in-place --expression \
'$i source "/opt/mavros/install/setup.bash"' /ros_entrypoint.sh && \
sed --in-place --expression \
'$i export PYTHONPATH=\$PYTHONPATH:/opt/gazebo/install/lib/python' /ros_entrypoint.sh && \
sed --in-place --expression \
'$i export PATH=/opt/ardupilot_dave/ardupilot/build/still/bin:\$PATH' /ros_entrypoint.sh && \
sed --in-place --expression \
'$i export PATH=/opt/ardupilot_dave/ardupilot/Tools/autotest:\$PATH' /ros_entrypoint.sh && \
sed --in-place --expression \
'$i export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_dave/ardupilot_gazebo/build:\$GZ_SIM_SYSTEM_PLUGIN_PATH' /ros_entrypoint.sh && \
sed --in-place --expression \
'$i export GZ_SIM_RESOURCE_PATH=/opt/ardupilot_dave/ardupilot_gazebo/models:/opt/ardupilot_dave/ardupilot_gazebo/worlds:\$GZ_SIM_RESOURCE_PATH' /ros_entrypoint.sh

# Set User as user
USER $USER
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc  && \
    echo "source /opt/gazebo/install/setup.bash" >> ~/.bashrc && \
    echo "source /opt/mavros/install/setup.bash" >> ~/.bashrc && \
    echo "export PYTHONPATH=\$PYTHONPATH:/opt/gazebo/install/lib/python" >> ~/.bashrc && \
    echo "export PATH=/opt/ardupilot_dave/ardupilot/build/still/bin:\$PATH" >> ~/.bashrc && \
    echo "export PATH=/opt/ardupilot_dave/ardupilot/Tools/autotest:\$PATH" >> ~/.bashrc && \
    echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_dave/ardupilot_gazebo/build:\$GZ_SIM_SYSTEM_PLUGIN_PATH" >> ~/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=/opt/ardupilot_dave/ardupilot_gazebo/models:/opt/ardupilot_dave/ardupilot_gazebo/worlds:\$GZ_SIM_RESOURCE_PATH" >> ~/.bashrc

