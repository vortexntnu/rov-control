FROM ros:kinetic-ros-base

RUN apt-get -y update && \
    apt-get install -y libeigen3-dev && \
    apt-get install -y ros-kinetic-roslint && \
    apt-get install -y ros-kinetic-eigen-conversions && \
    apt-get install -y ros-kinetic-tf-conversions && \
    cd usr/src/gtest && \
    cmake . && \
    make && \
    cp libg* /usr/lib/

WORKDIR /catkin_ws/src
RUN bash -c "source /opt/ros/kinetic/setup.bash && catkin_init_workspace"
