#! /usr/bin/bash

docker build --rm -t erl/ros1-bridge:latest . \
    --build-arg ROS1_IMAGE=erl/ros-noetic:cpu-common
