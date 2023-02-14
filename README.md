# LUNABOTICS-2023

The official NASA RMC 2023 repository for LUNABOTICS team, a segment of University of Minnesota Robotics.

## How to Run inside Docker Container

Open this repository in vscode then run ctrl-shift-p and type "Remote-Containers: Reopen in Container".
Just press "from dockerfile" and then it will build the container and run it.

When open, run the following commands in the terminal:

```
. /opt/ros/$ROS_DISTRO/setup.sh
colcon build
. install/setup.sh
```

## ROS 2 Eloquent General Workspace Tips

Make sure to `source install/setup.bash`

Install Dependencies is `rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y`

Clean workspace is `rm -r build install log`

## Useful Resources/References

Configuring the Nvidia Jetson TX2 for CAN Communication: 

1) https://www.mathworks.com/help/supportpkg/nvidia/ug/jetson-can-bus-traffic-sign-detection.html

2) https://forums.developer.nvidia.com/t/how-to-use-can-on-jetson-tx2/54125

VESC CAN Status Frames Spreadsheet: https://github.com/codermonkey42/VESC_CAN

Start the joystick node: `ros2 run joy joy_node`

Start the webcam node: `ros2 run v4l2_camera v4l2_camera_node --video_device /dev/video0`

View the webcam stream: `ros2 run rqt_image_view rqt_image_view`
