#!/bin/bash

source install/setup.bash && \
ros2 launch isaac_ros_launch EVERYTHING_launch.py run_rviz:=false from_bag:=false