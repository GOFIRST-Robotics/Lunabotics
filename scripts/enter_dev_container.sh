#!/bin/bash

# Check if the container is running
if [ "$(docker ps -q -f name=lunabotics_dev)" ]; then
    echo "Entering the lunabotics_dev container..."
    docker exec -it --workdir /workspaces/isaac_ros-dev lunabotics_dev bash
else
    echo "The container 'lunabotics_dev' is not running. Please run your container start script first."
fi
