FROM osrf/ros:foxy-desktop

ARG WORKSPACE=/ros2_ws

RUN mkdir ${WORKSPACE}

WORKDIR ${WORKSPACE}

RUN mkdir -p src/py_pubsub

COPY py_pubsub src/py_pubsub

RUN cat << EOF > publisher
COPY publisher .
COPY subscriber .

RUN ln -s ${WORKSPACE}/publisher /usr/local/bin/publisher
RUN ln -s ${WORKSPACE}/subscriber /usr/local/bin/subscriber

RUN colcon build --symlink-install
