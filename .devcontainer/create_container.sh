#!/bin/bash

docker build -t umnrobotics/cuda - < DOCKERFILE.cuda \
    --build-arg UBUNTU_RELEASE_YEAR=22 \
    --build-arg CUDA_MAJOR=12 --build-arg CUDA_MINOR=1 \
    --build-arg CUDA_PATCH=0 || exit

docker build -t umnrobotics/ros:cuda-humble - < DOCKERFILE.ros \
    --build-arg BASE=umnrobotics/cuda \
    --build-arg ROS_DISTRO=humble \
    --build-arg ROS_TYPE=desktop-full || exit

# zed is dumb so you have to manyally update tag of ros wrapper version
docker build -t umnrobotics/ros:humble-zed - < DOCKERFILE.zed \
    --build-arg BASE=umnrobotics/ros:cuda-humble \
    --build-arg UBUNTU_RELEASE_YEAR=22 \
    --build-arg CUDA_MAJOR=12 --build-arg CUDA_MINOR=1 \
    --build-arg CUDA_PATCH=0 --build-arg ZED_SDK_MAJOR=4 \
    --build-arg ZED_SDK_MINOR=0 || exit

docker build -t umnrobotics/ros:humble-zed-dev - < DOCKERFILE.dev \
    --build-arg BASE=umnrobotics/ros:humble-zed \
    --build-arg ROS_DISTRO=humble-desktop-full || exit
