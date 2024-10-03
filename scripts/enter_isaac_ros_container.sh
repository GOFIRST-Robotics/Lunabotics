#!/bin/bash
printf 'CONFIG_DOCKER_SEARCH_DIRS="$HOME/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/../../../../docker"\n'> ~/.isaac_ros_common-config
image_key="ros2_humble.realsense.deepstream.user.zed.umn.gazebo"

bash ~/Lunabotics/scripts/build_image.sh

echo "-v /usr/local/zed/resources:/usr/local/zed/resources -v /ssd:/ssd -v /usr/local/zed/settings:/usr/local/zed/settings" > ~/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/.isaac_ros_dev-dockerargs && \

bash ~/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/run_dev.sh -d ~/Lunabotics -i "${image_key}" -a "${docker_arg}" -v -b
