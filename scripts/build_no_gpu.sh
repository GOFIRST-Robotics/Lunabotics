#!/bin/bash

cd /workspaces/isaac_ros-dev || { echo "Failure: Could not find /workspaces/Lunabotics"; exit 1; }

colcon build --symlink-install --packages-up-to zig_test

