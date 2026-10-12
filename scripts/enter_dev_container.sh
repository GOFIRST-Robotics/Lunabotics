#!/bin/bash
set -e

# Default values
USE_GPU=false
ACTION="enter"

usage() {
    echo "Usage: $0 [-g] [-r] [-h]"
    echo "  -g    Enable GPU support"
    echo "  -r    Rebuild container (removes existing container and rebuilds image)"
    echo "  -h    Display help"
    exit 1
}

# Parse options
while getopts "grh" opt; do
    case ${opt} in
        g) USE_GPU=true ;;
        r) ACTION="rebuild" ;;
        h) usage ;;
        \?) echo "Invalid option: -${OPTARG}" >&2; usage ;;
    esac
done
shift $((OPTIND - 1))

# Set container and image names based on GPU flag
if [ "$USE_GPU" = true ]; then
    CONTAINER_NAME="lunabotics_gpu_dev"
    IMAGE_NAME="test-zed"
else
    CONTAINER_NAME="lunabotics_cpu_dev"
    IMAGE_NAME="lunabotics_dev"
fi

echo "GPU Enabled: $USE_GPU"
echo "Action:      $ACTION"
echo "Container:   $CONTAINER_NAME"
echo "Image:       $IMAGE_NAME"

# 1. Handle Rebuild or Missing Image
IMAGE_EXISTS=$(docker images -q "$IMAGE_NAME" 2> /dev/null)

if [ "$ACTION" = "rebuild" ] || [ -z "$IMAGE_EXISTS" ]; then
    echo "--> Rebuilding image/container..."
    
    # Remove existing container if it exists
    if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo "Removing existing container ${CONTAINER_NAME}..."
        docker rm -f "$CONTAINER_NAME"
    fi

    if [ "$USE_GPU" = true ]; then
        echo "Building GPU Image..."
        docker build \
            --no-cache \
            --build-arg BASE_IMAGE=docker.io/nvidia/cuda:13.2.1-cudnn-devel-ubuntu24.04 \
            -f docker/Dockerfile.dev \
            -t "lunabotics_dev" .

        docker build \
            --no-cache \
            --build-arg BASE_IMAGE=lunabotics_dev \
            --build-arg CUDA_MAJOR=13 \
            --build-arg CUDA_MINOR=2 \
            --build-arg ZED_SDK_MAJOR=5 \
            --build-arg ZED_SDK_MINOR=5 \
            -f docker/Dockerfile.zed.amd64 \
            -t "$IMAGE_NAME" .
    else
        echo "Building CPU Image..."
        docker build \
            --no-cache \
            --build-arg BASE_IMAGE=docker.io/ubuntu:24.04 \
            -f docker/Dockerfile.dev \
            -t "$IMAGE_NAME" .
    fi
fi

# 2. X11 Forwarding Setup
xhost +local:root > /dev/null

# 3. Attach, Start, or Run Container
if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "--> Attaching to existing running container: ${CONTAINER_NAME}"
    docker exec -it "$CONTAINER_NAME" /bin/bash

elif docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "--> Starting existing stopped container: ${CONTAINER_NAME}"
    docker start -ai "$CONTAINER_NAME"

else
    echo "--> Creating and starting new container: ${CONTAINER_NAME}"
    
    # Common flags shared across both CPU and GPU execution
    COMMON_RUN_ARGS=(
        -it
        --name "$CONTAINER_NAME"
        --privileged
        -e DISPLAY="$DISPLAY"
        -e FASTFASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml
        -v "$(pwd -P):/workspaces/isaac_ros-dev"
        -w /workspaces/isaac_ros-dev
        -v /dev/input:/dev/input
    )

    if [ "$USE_GPU" = true ]; then
        docker run "${COMMON_RUN_ARGS[@]}" \
            --gpus all \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -v /dev/bus/usb:/dev/bus/usb \
            -v /usr/local/zed/resources:/usr/local/zed/resources \
            -v /usr/local/zed/settings:/usr/local/zed/settings \
            "$IMAGE_NAME" \
            /bin/bash
    else
        docker run "${COMMON_RUN_ARGS[@]}" \
            --cap-add NET_ADMIN \
            --network host \
            "$IMAGE_NAME" \
            /bin/bash
    fi
fi