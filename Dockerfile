FROM osrf/ros:humble-desktop

RUN mkdir -p ros_ws/src

WORKDIR /ros_ws/src

COPY . .

WORKDIR /ros_ws
