#!/bin/bash

CONTAINER_NAME=lunabotics_dev
FORCE_REBUILD="$1"

if [ "$FORCE_REBUILD" = "false" ] || [[ "$(docker images -q $CONTAINER_NAME 2> /dev/null)" == "" ]]; then
    if [ "$FORCE_REBUILD" = "false" ]; then
        echo "Argument 'false' provided. Forcing image rebuild..."
    else
        echo "Image '$CONTAINER_NAME' not found. Building..."
    fi
    docker build -f docker/Dockerfile.dev -t $CONTAINER_NAME .
else
    echo "Image '$CONTAINER_NAME' already exists and rebuild not requested. Skipping build."
fi

if [ "$(docker ps -q -f name=^/${CONTAINER_NAME}$)" ]; then
    echo "Container '$CONTAINER_NAME' is running. Connecting..."
    docker exec -it --workdir /workspaces/isaac_ros-dev $CONTAINER_NAME /bin/bash
elif [ "$(docker ps -aq -f status=exited -f name=^/${CONTAINER_NAME}$)" ]; then
    echo "Container '$CONTAINER_NAME' exists but is stopped. Starting..."
    docker start $CONTAINER_NAME
    docker exec -it --workdir /workspaces/isaac_ros-dev $CONTAINER_NAME /bin/bash
else
    echo "Creating and starting new container '$CONTAINER_NAME'..."
    # Always clear potential stale or non-running containers with the same name
    docker rm -f $CONTAINER_NAME 2>/dev/null || true

    docker run -it \
        --name $CONTAINER_NAME \
        --cap-add NET_ADMIN \
        --network host \
        --privileged \
        -e DISPLAY=$DISPLAY \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e FASTFASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml \
        -v $(pwd -P):/workspaces/isaac_ros-dev \
        -w /workspaces/isaac_ros-dev \
        -v "/dev/input:/dev/input" \
        $CONTAINER_NAME \
        /bin/bash
fi