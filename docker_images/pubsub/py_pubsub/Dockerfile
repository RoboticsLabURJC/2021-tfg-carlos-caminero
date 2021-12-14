FROM osrf/ros:foxy-desktop

ARG WORKSPACE=/ros2_ws

RUN mkdir ${WORKSPACE}

WORKDIR ${WORKSPACE}

RUN mkdir src
RUN colcon build --symlink-install

COPY py_pubsub src
