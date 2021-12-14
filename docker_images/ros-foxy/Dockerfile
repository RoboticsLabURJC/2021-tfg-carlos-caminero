FROM osrf/ros:foxy-desktop

ARG WORKSPACE=/ros2_ws

# Installing initial packages
RUN apt-get update
RUN apt-get install -y wget apt-utils python3-distutils python3-apt xterm
RUN apt-get install -y ros-$ROS_DISTRO-action-msgs
RUN apt-get install -y ros-$ROS_DISTRO-ament-cmake


# Creating workspace
RUN mkdir -p ${WORKSPACE}/src

# Installing darknet_ros for ROS2
RUN cd ${WORKSPACE}/src && git clone https://github.com/Ar-Ray-code/darknet_ros_fp16.git
RUN cd ${WORKSPACE}/src/darknet_ros_fp16 && git clone --recursive https://github.com/Ar-Ray-code/darknet_ros.git
RUN cd ${WORKSPACE}/src && darknet_ros_fp16/darknet_ros/rm_darknet_CMakeLists.sh
RUN rosdep update && cd ${WORKSPACE} && rosdep install --from-paths src --ignore-src -r -y

# Compiling packages
RUN . /opt/ros/foxy/setup.sh && cd ${WORKSPACE} && colcon build --symlink-install && . ${WORKSPACE}/install/setup.sh


