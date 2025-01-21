#!/bin/bash
export image_key="ros2_humble.zed.umn.gazebo"
docker_arg="-v /usr/local/zed/resources:/usr/local/zed/resources -v $HOME/rosbags:/rosbags -v /usr/local/zed/settings:/usr/local/zed/settings"

USE_CACHED_IMAGE=${1:-true}

if $USE_CACHED_IMAGE && docker images | grep -q "${image_key}"; then
    echo "Image ${image_key} already exists"
else
    echo "Building image ${image_key}"
    bash ~/Lunabotics/scripts/build_image.sh $USE_CACHED_IMAGE
fi

bash ~/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/run_dev.sh -d ~/Lunabotics -i "${image_key}" -a "${docker_arg}" -v -b
