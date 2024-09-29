#!/bin/bash
printf 'CONFIG_DOCKER_SEARCH_DIRS="$HOME/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/../../../../docker"\n'> ~/.isaac_ros_common-config
docker_arg="-v /usr/local/zed/resources:/usr/local/zed/resources -v /ssd:/ssd -v /usr/local/zed/settings:/usr/local/zed/settings"
bash ~/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/run_dev.sh -d ~/Lunabotics -i "ros2_humble.realsense.deepstream.user.zed1.umn.gazebo" -a "${docker_arg}" -v