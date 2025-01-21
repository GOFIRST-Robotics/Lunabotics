#!/bin/bash
if [ -z "${image_key}" ]; then
    image_key="ros2_humble.zed.umn.gazebo"
fi

PLATFORM="$(uname -m)"

#Write the config file for the build_image_layers.sh script
printf 'CONFIG_DOCKER_SEARCH_DIRS="$HOME/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/../../../../docker"\nBASE_DOCKER_REGISTRY_NAMES=("nvcr.io/nvidia/isaac/ros" "umnrobotics/isaac_ros3.2")\n'> ~/.isaac_ros_common-config

USE_CACHED_IMAGE=${1:-true}
if [ "${USE_CACHED_IMAGE}" = "false" ]; then
    echo "Building image locally"
    #Does not attempt to pull the image from remote
    echo "SKIP_REGISTRY_CHECK=1" >> ~/.isaac_ros_common-config
else
    echo "Using remote image"
fi

#Copy the provided zed install script (from isaac_ros) to the docker directory
if [ ! -f ${HOME}/Lunabotics/docker/zed/install-zed.sh ]; then
    mkdir -p "${HOME}/Lunabotics/docker/zed"
    if [[ $PLATFORM == "x86_64" ]]; then
        cp "${HOME}/Lunabotics/src/isaac_ros/isaac_ros_common/docker/scripts/install-zed-x86_64.sh" "${HOME}/Lunabotics/docker/zed/install-zed.sh"
    else
        cp "${HOME}/Lunabotics/src/isaac_ros/isaac_ros_common/docker/scripts/install-zed-aarch64.sh" "${HOME}/Lunabotics/docker/zed/install-zed.sh"
    fi
    chmod +x "${HOME}/Lunabotics/docker/zed/install-zed.sh"
fi 

#Download the deepstream deb file
if [ ! -f ${HOME}/Lunabotics/docker/deepstream/deepstream*.deb ]; then
    ngc registry resource download-version nvidia/deepstream:7.1 --dest "${HOME}/Lunabotics/docker/deepstream" --file "*${ARCH}.deb" || echo You need to install ngc!
fi

#Rename the downloaded deepstream file to deepstream.deb
find "${HOME}/Lunabotics/docker/deepstream" -name *${ARCH}.deb -type f | xargs -I f mv f ${HOME}/Lunabotics/docker/deepstream/deepstream.deb

#Build the image finally
bash ~/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/build_image_layers.sh --ignore_composite_keys --image_key "${PLATFORM}.${image_key}" --image_name "isaac_ros_dev-$PLATFORM" || exit 1
