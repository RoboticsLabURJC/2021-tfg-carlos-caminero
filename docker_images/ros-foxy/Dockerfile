FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu20.04
ARG VIRTUALGL_VERSION=2.6.5
ARG TURBOVNC_VERSION=2.2.80

ARG DEBIAN_FRONTEND=noninteractive
ARG WORKSPACE=/ros2_ws

# Install CLI tools
RUN apt-get update && apt-get install -y \
  curl \
  lsb-release \
  python3 python3-rosdep \
  git

# Install ROS
ENV ROS_DISTRO foxy
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null' \
  && apt-get update \
  && apt-get -y --quiet --no-install-recommends install ros-$ROS_DISTRO-desktop

# Install ROS packages
RUN apt-get install -y \
  python3-colcon-common-extensions \
  ros-$ROS_DISTRO-gazebo-* \
  ros-$ROS_DISTRO-cartographer \
  ros-$ROS_DISTRO-cartographer-ros

# Creating workspace
RUN mkdir -p ${WORKSPACE}/src
WORKDIR ${WORKSPACE}
RUN colcon build --symlink-install



