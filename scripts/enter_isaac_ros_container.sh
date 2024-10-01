#!/bin/bash
printf 'CONFIG_DOCKER_SEARCH_DIRS="$HOME/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/../../../../docker"\n'> ~/.isaac_ros_common-config
image_key="ros2_humble.realsense.deepstream.user1.zed1.umn.gazebo"
docker_arg="-v /usr/local/zed/resources:/usr/local/zed/resources -v /ssd:/ssd -v /usr/local/zed/settings:/usr/local/zed/settings"
if [ "$(uname -m)" = "x86_64" ]; then
    ARCH=amd64
else
    ARCH=amd64
fi
if [ ! -f deepstream/*/*${ARCH}.deb ]; then
    ngc registry resource download-version nvidia/deepstream:7.0 --dest 'deepstream' --file "*${ARCH}*" || (echo You need to install ngc! && exit 1)
fi
bash ~/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/run_dev.sh -d ~/Lunabotics -i "${image_key}" -a "${docker_arg}" -v
