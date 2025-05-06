#!/bin/bash
printf 'CONFIG_DOCKER_SEARCH_DIRS="$HOME/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/../../../../docker"\n BASE_DOCKER_REGISTRY_NAMES="umnrobotics/isaac_ros3.1"\n'> ~/.isaac_ros_common-config
image_key="ros2_humble.deepstream.user.zed.umn.gazebo.streamdeck"
docker_arg="-v /usr/local/zed/resources:/usr/local/zed/resources -v $HOME/rosbags:/rosbags -v /usr/local/zed/settings:/usr/local/zed/settings --privileged"

USE_CACHED_IMAGE=${1:-true}

if $USE_CACHED_IMAGE && docker images | grep -q "${image_key}"; then
    echo "Image ${image_key} already exists"
else
    echo "Building image ${image_key}"
    bash ~/Lunabotics/scripts/build_image.sh $USE_CACHED_IMAGE
fi

bash ~/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/run_dev.sh -d ~/Lunabotics -i "${image_key}" -a "${docker_arg}" -v -b
