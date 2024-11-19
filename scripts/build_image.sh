
if [ -z "${image_key}"]; then
    image_key="ros2_humble.realsense.deepstream.user.zed.umn.gazebo"
fi

declare -a arr=("$(uname -m)")

for PLAT in "${arr[@]}"
do

if [ "${PLAT}" = "x86_64" ]; then
    ARCH=amd64
else
    ARCH=arm64
fi

USE_CACHED_IMAGE=${1:-true}
if [ "${USE_CACHED_IMAGE}" = "false" ]; then
    echo "Building image locally"
    printf 'CONFIG_DOCKER_SEARCH_DIRS="$HOME/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/../../../../docker"\n BASE_DOCKER_REGISTRY_NAMES=""\n'> ~/.isaac_ros_common-config
else
    echo "Using remote image"
    printf 'CONFIG_DOCKER_SEARCH_DIRS="$HOME/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/../../../../docker"\n BASE_DOCKER_REGISTRY_NAMES="umnrobotics/isaac_ros3.1"\n'> ~/.isaac_ros_common-config
fi

if [ ! -f ${HOME}/Lunabotics/docker/deepstream/deepstream*.deb ]; then
    ngc registry resource download-version nvidia/deepstream:7.0 --dest "${HOME}/Lunabotics/docker/deepstream" --file "*${ARCH}.deb" || echo You need to install ngc!
fi

find "${HOME}/Lunabotics/docker/deepstream" -name *${ARCH}.deb -type f | xargs -I f mv f ${HOME}/Lunabotics/docker/deepstream/deepstream.deb
bash ~/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/build_image_layers.sh --ignore_composite_keys -a "ZED_SDK_MINOR=2" -a "ARCH=${ARCH}" -i "${PLAT}.${image_key}" -n "isaac_ros_dev-${PLAT}" || exit 1

done