FROM ros:humble-ros-base

LABEL Maintainer="Jakub Czech <czechjakub@icloud.com>"
LABEL Description="Kinco AC Servo driver for ROS2"

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN ./ros_entrypoint.sh
RUN apt-get update && apt-get upgrade -y && apt-get autoremove -y

RUN apt-get update && apt-get install -y \
    python3-pip 
RUN pip3 install -U \
    pyserial 
#     python3-colcon-common-extensions
RUN mkdir -p /root/workspace/src
WORKDIR /root/workspace
COPY ./src_files/src ./src
RUN colcon build --symlink-install --packages-select kinco_driver