# LUNABOTICS-2023

The NASA RMC 2023 repository for LUNABOTICS team, a segment of University of Minnesota Robotics.

## ROS2 Humble Windows 10 install tips

Follow this guide, but read the notes below first.
<https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html>

You DO need to install VS Code, even if you have it already, since we need to add specific build components that you probably didn’t opt-in to installing last time.

Even if you already have a newer version of python, you do need to install python 3.8 or whatever version is listed in the first line of C:\dev\ros2-windows\Scripts\ros2-script.py. Any other version will result in ros2 failing to launch.

You need to run C:\dev\ros2-windows\local_setup.bat every time you start a new command prompt. Make a batch script for it.

## ROS2 General Workspace Tips

Make sure to source install/setup.bash

Clean workspace is rm -r build install log

## Useful ROS2 Commands for Testing

Install Dependencies: rosdep install -i --from-path src --rosdistro humble -y

Source the setup files: . install/setup.bash
