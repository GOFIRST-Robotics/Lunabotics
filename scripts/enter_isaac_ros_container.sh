#!/bin/bash
printf 'CONFIG_DOCKER_SEARCH_DIRS="$HOME/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/../../../../docker"\n'> ~/.isaac_ros_common-config
image_key="ros2_humble.realsense.deepstream.user.zed.umn.gazebo"
docker_arg="-v /usr/local/zed/resources:/usr/local/zed/resources -v /ssd:/ssd -v /usr/local/zed/settings:/usr/local/zed/settings"
if [ "$(uname -m)" = "x86_64" ]; then
    ARCH=amd64
else
    ARCH=arm64
fi

if [ ! -f ${HOME}/Lunabotics/docker/deepstream/deepstream.deb ]; then
    ngc registry resource download-version nvidia/deepstream:7.0 --dest "${HOME}/Lunabotics/docker/deepstream" --file "*${ARCH}*" || echo You need to install ngc!
    find "${HOME}/Lunabotics/docker/deepstream" -name *.deb -type f | xargs -I f mv f ${HOME}/Lunabotics/docker/deepstream/deepstream.deb
fi
bash ~/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/build_image_layers.sh -a "ZED_SDK_MINOR=1" -a "ARCH=${ARCH}" -i "${image_key}"
bash ~/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/run_dev.sh -d ~/Lunabotics -i "${image_key}" -a "${docker_arg}" -v
