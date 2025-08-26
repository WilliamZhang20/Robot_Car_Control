# ROS2 humble Base
FROM ros:humble

RUN apt-get update || true && apt-get install -y curl

RUN rm -f /etc/apt/sources.list.d/ros2.list \
    /etc/apt/sources.list.d/ros2-latest.list \
    /etc/apt/sources.list.d/ros-latest.list && \
    rm -f /usr/share/keyrings/ros2-latest-archive-keyring.gpg \
    /usr/share/keyrings/ros2-archive-keyring.gpg \
    /usr/share/keyrings/ros2-latest-archive-keyring.gpg

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update # Run apt-get update again after updating the key

RUN apt-get upgrade -y

# https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/docker-specialized.html
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

RUN apt-get install --no-install-recommends -y \
    software-properties-common \
    vim \
    python3-pip \
    python3-tk

# Added updated mesa drivers for integration with cpu - https://github.com/ros2/rviz/issues/948#issuecomment-1428979499
RUN add-apt-repository ppa:kisak/kisak-mesa && \
    apt-get update && apt-get upgrade -y

# Cyclone DDS
RUN apt-get install --no-install-recommends -y \
    ros-$ROS_DISTRO-cyclonedds \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp

# Use cyclone DDS by default
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source by default
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

USER root

ENV WORKSPACE_PATH=/root/workspace

ENV IGN_RENDERING_API=ogre

RUN apt-get update && apt-get install -y mesa-utils

COPY workspace/ $WORKSPACE_PATH/src/

RUN apt-get update && apt-get install -y ros-humble-xacro

# Install rosdep dependencies and other packages
RUN apt-get update && rosdep update && cd $WORKSPACE_PATH && \
    rosdep install --from-paths src -y --ignore-src

RUN apt-get update && apt-get install -y \
    ros-humble-teleop-twist-keyboard \
    ros-humble-gazebo-ros-pkgs \
    x11-apps

COPY scripts/setup/ /root/scripts/setup
RUN /root/scripts/setup/workspace.sh