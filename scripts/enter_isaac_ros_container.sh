#!/bin/bash
printf 'CONFIG_DOCKER_SEARCH_DIRS="$HOME/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/../../../../docker"\n BASE_DOCKER_REGISTRY_NAMES=""\n'> ~/.isaac_ros_common-config
image_key="ros2_humble.realsense.deepstream.user.zed.umn.gazebo"
docker_arg="-v /usr/local/zed/resources:/usr/local/zed/resources -v $HOME/rosbags:/rosbags -v /usr/local/zed/settings:/usr/local/zed/settings"

bash ~/Lunabotics/scripts/build_image.sh

bash ~/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/run_dev.sh -d ~/Lunabotics -i "${image_key}" -a "${docker_arg}" -v -b
