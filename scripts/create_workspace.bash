#!/usr/bin/bash

WORKSPACE_DIR=${WORKSPACE_DIR:-"ros_ws_common"}
mkdir -p ${WORKSPACE_DIR}/src
cd ${WORKSPACE_DIR}/src
for repo in erl_cmake_tools \
            erl_common \
            erl_common_ros; do
    if [ ! -d ${repo} ]; then
        git clone https://github.com/ExistentialRobotics/${repo}.git -b main
    else
        echo "Repository ${repo} already exists, skipping clone."
    fi
done
