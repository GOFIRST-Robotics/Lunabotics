#!/bin/bash
printf 'CONFIG_DOCKER_SEARCH_DIRS="$HOME/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/../../../../docker"\n BASE_DOCKER_REGISTRY_NAMES="umnrobotics/isaac_ros3.1"\n'> ~/.isaac_ros_common-config
image_key="ros2_humble.realsense.deepstream.user.zed.umn.gazebo"
docker_arg="-v /usr/local/zed/resources:/usr/local/zed/resources -v $HOME/rosbags:/rosbags -v /usr/local/zed/settings:/usr/local/zed/settings"

if docker images | grep -q "${image_key}"; then
    echo "Image ${image_key} already exists"
else
    echo "Building image ${image_key}"
    bash ~/Lunabotics/scripts/build_image.sh
fi

bash ~/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/run_dev.sh -d ~/Lunabotics -i "${image_key}" -a "${docker_arg}" -v -b
