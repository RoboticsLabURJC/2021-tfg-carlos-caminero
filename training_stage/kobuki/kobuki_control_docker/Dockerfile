FROM osrf/ros:foxy-desktop

ARG WORKSPACE=/ros2_ws

# Installing initial packages
RUN apt-get update
RUN apt-get install -y apt-utils python3-distutils python3-apt xterm
RUN apt-get install -y ros-$ROS_DISTRO-action-msgs
RUN apt-get install -y ros-$ROS_DISTRO-ament-cmake

# Creating workspace
RUN mkdir -p ${WORKSPACE}/src

# Installing kobuki_ros packages and dependencies
RUN git clone --recursive https://github.com/Carlosalpha1/kobuki_ros.git && mv kobuki_ros ${WORKSPACE}/src
RUN git clone --recursive https://github.com/kobuki-base/kobuki_ros_interfaces.git && mv kobuki_ros_interfaces ${WORKSPACE}/src
RUN rosdep update && cd ${WORKSPACE} && rosdep install --from-paths src --ignore-src -r -y

# Compiling packages
RUN . /opt/ros/foxy/setup.sh && cd ${WORKSPACE} && colcon build --symlink-install && . ${WORKSPACE}/install/setup.sh

# Start Command
CMD . /ros2_ws/install/setup.sh && ros2 launch kobuki_keyop kobuki_keyop.launch.py
