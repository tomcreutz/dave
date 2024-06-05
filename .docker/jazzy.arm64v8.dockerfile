FROM arm64v8/ubuntu:24.04
# Set RDP and SSH environments
ARG X11Forwarding=true
# hadolint ignore=DL3008,DL3015,DL3009
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
        apt-get install -y ubuntu-desktop-minimal=1.539 dbus-x11 xrdp sudo; \
    [ $X11Forwarding = 'true' ] && apt-get install -y openssh-server; \
    apt-get autoremove --purge; \
    apt-get clean; \
    rm /run/reboot-required*

ARG USER=docker
ARG PASS=docker

RUN useradd -s /bin/bash -m $USER -p "$(openssl passwd "$PASS")"; \
    usermod -aG sudo $USER; echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers; \
    adduser xrdp ssl-cert; \
    # Setting the required environment variables
    echo 'LANG=en_US.UTF-8' >> /etc/default/locale; \
    echo 'export GNOME_SHELL_SESSION_MODE=ubuntu' > /home/$USER/.xsessionrc; \
    echo 'export XDG_CURRENT_DESKTOP=ubuntu:GNOME' >> /home/$USER/.xsessionrc; \
    echo 'export XDG_SESSION_TYPE=x11' >> /home/$USER/.xsessionrc; \
    # Enabling log to the stdout
    sed -i "s/#EnableConsole=false/EnableConsole=true/g" /etc/xrdp/xrdp.ini; \
    # Disabling system animations and reducing the
    # image quality to improve the performance
    sed -i 's/max_bpp=32/max_bpp=16/g' /etc/xrdp/xrdp.ini; \
    gsettings set org.gnome.desktop.interface enable-animations true; \
    # Listening on wildcard address for X forwarding
    [ $X11Forwarding = 'true' ] && \
        sed -i 's/#X11UseLocalhost yes/X11UseLocalhost no/g' /etc/ssh/sshd_config || \
        sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/g' /etc/ssh/sshd_config || \
        :;

# Disable initial welcome window
RUN echo "X-GNOME-Autostart-enabled=false" >> /etc/xdg/autostart/gnome-initial-setup-first-login.desktop

# Run
EXPOSE 3389/tcp
EXPOSE 22/tcp
# hadolint ignore=DL3025
CMD sudo rm -f /var/run/xrdp/xrdp*.pid >/dev/null 2>&1; \
    sudo service dbus restart >/dev/null 2>&1; \
    sudo /usr/lib/systemd/systemd-logind >/dev/null 2>&1 & \
    [ -f /usr/sbin/sshd ] && sudo /usr/sbin/sshd; \
    sudo xrdp-sesman --config /etc/xrdp/sesman.ini; \
    sudo xrdp --nodaemon --config /etc/xrdp/xrdp.ini


# ROS-Gazebo arg
ARG BRANCH="ros2"
ARG ROS_DISTRO="jazzy"

# Install basics
USER root
ENV DEBIAN_FRONTEND noninteractive
ENV DEBCONF_NONINTERACTIVE_SEEN=true
# hadolint ignore=DL3008
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    sudo xterm init systemd snapd vim net-tools \
    curl wget git build-essential cmake cppcheck \
    gnupg libeigen3-dev libgles2-mesa-dev \
    lsb-release pkg-config protobuf-compiler \
    python3-dbg python3-pip python3-venv \
    qtbase5-dev ruby dirmngr gnupg2 nano xauth \
    software-properties-common htop libtool \
    x11-apps mesa-utils bison flex automake && \
    rm -rf /var/lib/apt/lists/

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


# Install ROS-Gazebo framework
ADD https://raw.githubusercontent.com/IOES-Lab/dave/$BRANCH/\
extras/ros-jazzy-gz-harmonic-install.sh install.sh
RUN bash install.sh

# Set up Dave workspace
ENV ROS_UNDERLAY /home/$USER/dave_ws/install
WORKDIR $ROS_UNDERLAY/../src

ADD https://raw.githubusercontent.com/IOES-Lab/dave/$BRANCH/\
extras/repos/dave.$ROS_DISTRO.repos /home/$USER/ws_dave/dave.repos
RUN vcs import --shallow --input "/home/$USER/ws_dave/dave.repos"

RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

RUN apt-get update && rosdep update && \
    rosdep install -iy --from-paths . && \
    rm -rf /var/lib/apt/lists/

WORKDIR $ROS_UNDERLAY/..
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build

# source entrypoint setup
RUN touch /ros_entrypoint.sh && sed --in-place --expression \
    '$i source "$ROS_UNDERLAY/setup.bash"' /ros_entrypoint.sh

# Set User as user
USER docker

# Set volume previleage for HOST home directory
RUN echo "chown docker:docker ~/HOST" >> ~/.bashrc

# Use software rendering for container
ENV LIBGL_ALWAYS_INDIRECT=1
