#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 daemon stop
ros2 daemon start
ros2 topic echo joy