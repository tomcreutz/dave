FROM arm64v8/ubuntu:24.04
ARG BRANCH="ros2"
ARG ROS_DISTRO="jazzy"

EXPOSE 3389/tcp
# EXPOSE 22/tcp
ARG USER=ioes
ARG PASS=ioes
ARG X11Forwarding=false

# Set RDP and SSH environments
# access with any RDP client at localhost:3389 with USER/PASS)
# SSh connect and forward X11 with USER/PASS at localhost:22
# hadolint ignore=DL3008
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
        apt-get install -y --no-install-recommends \
        ubuntu-desktop-minimal dbus-x11 xrdp sudo && \
        rm -rf /var/lib/apt/lists/; \
    [ $X11Forwarding = 'true' ] && \
    apt-get install -y --no-install-recommends \
    openssh-server; \
    apt-get autoremove --purge; \
    apt-get clean; \
    rm /run/reboot-required*

    RUN useradd -s /bin/bash -m $USER -p "$(openssl passwd "$PASS")"; \
    usermod -aG sudo $USER; \
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

# Default command to start rdp server
CMD ["/bin/sh", "-c", "\
    rm -f /var/run/xrdp/xrdp*.pid >/dev/null 2>&1; \
    service dbus restart >/dev/null 2>&1; \
    /usr/lib/systemd/systemd-logind >/dev/null 2>&1 & \
    [ -f /usr/sbin/sshd ] && /usr/sbin/sshd; \
    xrdp-sesman --config /etc/xrdp/sesman.ini; \
    xrdp --nodaemon --config /etc/xrdp/xrdp.ini \
    "]

# update and upgrade libs
RUN apt-get update \
    && apt-get -y upgrade \
    && rm -rf /tmp/*

# Install basics
ENV DEBIAN_FRONTEND noninteractive
ENV DEBCONF_NONINTERACTIVE_SEEN=true
# hadolint ignore=DL3008
RUN apt-get update \
    apt-get install -y --no-install-recommends \
    sudo tzdata build-essential gfortran automake \
    bison flex libtool git wget locales \
    software-properties-common && \
    rm -rf /var/lib/apt/lists/

# Locale for UTF-8
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# Install Utilities
# hadolint ignore=DL3008
RUN apt-get -y install --no-install-recommends \
    x11-apps mesa-utils xauth && \
    rm -rf /var/lib/apt/lists/

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

# Use software rendering for container
ENV LIBGL_ALWAYS_INDIRECT=1
