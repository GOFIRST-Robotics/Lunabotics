#!/bin/bash
printf 'CONFIG_DOCKER_SEARCH_DIRS="$HOME/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/../../../../docker"\n'> ~/.isaac_ros_common-config
image_key="ros2_humble.realsense.user.zed1.umn.gazebo"
docker_arg="-v /usr/local/zed/resources:/usr/local/zed/resources -v /ssd:/ssd -v /usr/local/zed/settings:/usr/local/zed/settings"
bash ~/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/build_image_layers.sh -b nvcr.io/nvidia/deepstream:7.0-triton-multiarch -i "${image_key}"
bash ~/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/run_dev.sh -d ~/Lunabotics -i "${image_key} -a "${docker_arg}" -v