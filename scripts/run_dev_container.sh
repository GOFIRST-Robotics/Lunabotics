#!/bin/bash

if [ ! -f "docker/Dockerfile.dev" ]; then
    echo "ERROR: Dockerfile.dev not found in docker/Dockerfile.dev."
    exit 1
fi

echo "Building UMN Lunabotics Docker image..."
if ! docker build -f docker/Dockerfile.dev -t umn_lunabotics --build-arg HOST_PROJECT_PATH=$(pwd) . ; then
    echo "ERROR: Docker build failed! Stopping execution."
    exit 1
fi

# Run the Docker container
# -d runs it in detached mode
# --privileged and --network host were specified in runArgs
# docker build --build-arg HOST_PROJECT_PATH=$(pwd) -t isaac-ros-dev-zig .
echo "Starting container..."
docker run -d \
    --name lunabotics_dev \
    --privileged \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e FASTFASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml \
    -v $(pwd -P):/workspaces/isaac_ros-dev \
    -w /workspaces/isaac_ros-dev \
    umn_lunabotics sleep infinity

echo "Container is running! Use enter_container.sh to access it."
