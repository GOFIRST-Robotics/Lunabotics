#!/bin/bash

source install/setup.bash && \
ros2 launch gstreamer laptop_launch.py

sudo sh -c "echo 0 > /sys/bus/usb/devices/1-1/authorized"
sleep 0.2
sudo sh -c "echo 1 > /sys/bus/usb/devices/1-1/authorized"
