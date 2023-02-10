# LUNABOTICS-2023

The official NASA RMC 2023 repository for LUNABOTICS team, a segment of University of Minnesota Robotics.

## ROS 2 Eloquent Windows 10 install tips

Read the notes below, then follow this guide: <https://docs.ros.org/en/eloquent/Installation/Windows-Install-Binary.html>

You DO need to install VS Code, even if you have it already, since you need to add specific build components that you probably didn’t opt-in to installing last time.

Even if you already have a newer version of python, you do need to install python 3.8 or whatever version is listed in the first line of C:\dev\ros2-windows\Scripts\ros2-script.py. Any other version will result in ros2 failing to launch.

You need to run C:\dev\ros2-windows\local_setup.bat every time you start a new command prompt. Make a batch script for it.

## ROS 2 Eloquent General Workspace Tips

Make sure to source install/setup.bash

Clean workspace is rm -r build install log

## How to Run inside Docker Container

Open this repository in vscode then run ctrl-shift-p and type "Remote-Containers: Reopen in Container".
Just press "from dockerfile" and then it will build the container and run it.

When open, run the following commands in the terminal:

. /opt/ros/$ROS_DISTRO/setup.sh

colcon build

. install/setup.sh

## Useful Resources/References

Configuring the Nvidia Jetson TX2 for CAN Communication: 

1) https://www.mathworks.com/help/supportpkg/nvidia/ug/jetson-can-bus-traffic-sign-detection.html

2) https://forums.developer.nvidia.com/t/how-to-use-can-on-jetson-tx2/54125

VESC CAN Status Frames Spreadsheet: https://github.com/codermonkey42/VESC_CAN

Command for starting the joystick node: ros2 run joy joy_node
