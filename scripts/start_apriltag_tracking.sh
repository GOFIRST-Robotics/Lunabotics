#!/bin/bash

ros2 launch rovr_control realsense_launch.py > /dev/null 2>&1 &

cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
    ./scripts/run_dev.sh
