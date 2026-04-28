#!/bin/bash
printf 'CONFIG_DOCKER_SEARCH_DIRS="$HOME/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/../../../../docker"\n BASE_DOCKER_REGISTRY_NAMES="umnrobotics/isaac_ros3.1"\n'> ~/.isaac_ros_common-config
image_key="ros2_humble.deepstream.user.zed.umn.gazebo"

# Start with your existing volumes
docker_arg="-v /usr/local/zed/resources:/usr/local/zed/resources \
-v $HOME/rosbags:/rosbags \
-v /usr/local/zed/settings:/usr/local/zed/settings \
-v /dev/v4l:/dev/v4l \
-v /dev/video0:/dev/video0 \
-v /dev/video1:/dev/video1 \
-v /dev/video2:/dev/video2 \
-v /dev/video3:/dev/video3 \
-v /dev/video4:/dev/video4 \
-v /dev/video5:/dev/video5 \
-v /dev/video6:/dev/video6 \
-v /dev/video7:/dev/video7 \
-v /dev/video8:/dev/video8"

USE_CACHED_IMAGE=${1:-true}

if $USE_CACHED_IMAGE && docker images | grep -q "${image_key}"; then
    echo "Image ${image_key} already exists"
else
    echo "Building image ${image_key}"
    bash ~/Lunabotics/scripts/build_image.sh $USE_CACHED_IMAGE
fi

bash ~/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/run_dev.sh -d ~/Lunabotics -i "${image_key}" -a "${docker_arg}" -v -b
