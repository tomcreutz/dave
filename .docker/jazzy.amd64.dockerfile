ARG ROS_DISTRO="jazzy"
FROM osrf/ros:$ROS_DISTRO-desktop-full
ARG BRANCH="ros2"

ADD https://raw.githubusercontent.com/IOES-Lab/dave/$BRANCH/\
extras/ros-jazzy-gz-harmonic-install.sh install.sh
RUN bash install.sh

# Make user (assume host user has 1000:1000 permission)
ARG USER=dave
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN addgroup --gid 1000 $USER && \
    adduser --uid 1000 --ingroup $USER --shell /bin/bash --disabled-password --gecos '' $USER \
    && echo "$USER:$USER" | chpasswd && adduser $USER sudo \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
# Set User as user
USER $USER

# RUN adduser --shell /bin/bash --disabled-password --gecos "" user \
#     && echo 'user:user' | chpasswd && adduser user sudo \
#     && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

ENV ROS_UNDERLAY ~/home/$USER/ws_dave/install
WORKDIR $ROS_UNDERLAY/../src

ADD https://raw.githubusercontent.com/IOES-Lab/dave/$BRANCH/\
extras/repos/dave.$ROS_DISTRO.repos dave.repos
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
