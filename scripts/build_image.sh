
base_key="${PLAT}.ros2_humble.realsense.deepstream.user.zed.umn.gazebo"

if [ ! -f ${HOME}/Lunabotics/docker/deepstream/*/deepstream*.deb ]; then
    ngc registry resource download-version nvidia/deepstream:7.0 --dest "${HOME}/Lunabotics/docker/deepstream" --file "*.deb" || echo You need to install ngc!
fi

declare -a arr=("$(uname -m)")

for PLAT in "${arr[@]}"
do

if [ "${PLAT}" = "x86_64" ]; then
    ARCH=amd64
else
    ARCH=arm64
fi

find "${HOME}/Lunabotics/docker/deepstream" -name *${ARCH}.deb -type f | xargs -I f mv f ${HOME}/Lunabotics/docker/deepstream/deepstream.deb
bash ~/Lunabotics/src/isaac_ros/isaac_ros_common/scripts/build_image_layers.sh --ignore_composite_keys -a "ZED_SDK_MINOR=1" -a "ARCH=${ARCH}" -i "${PLAT}.${base_key}" -n "umnrobotics/isaac_ros" || exit 1

done