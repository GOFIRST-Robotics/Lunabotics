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

Start the joystick node with ROS parameters: `ros2 run joy joy_node --ros-args --params-file joy_node.yaml`

Start Gstreamer H265 Encoding (On Nvidia Jetson): `gst-launch-1.0 v4l2src device=/dev/video0 ! "video/x-raw,width=640,height=480,framerate=30/1" ! nvvidconv ! "video/x-raw(memory:NVMM),format=I420" ! omxh265enc bitrate=100000 ! "video/x-h265,stream-format=byte-stream" ! h265parse ! rtph265pay ! udpsink host=127.0.0.1 port=5000`

Start Gstreamer H265 Decoding (On Nvidia Jetson): `gst-launch-1.0 udpsrc port=5000 ! "application/x-rtp,payload=96" ! rtph265depay ! h265parse ! omxh265dec ! nvvidconv ! xvimagesink`

Start Gstreamer H265 Decoding (On Ubuntu Laptop): `gst-launch-1.0 -v udpsrc port=5000 ! h265parse ! vaapih265dec ! vaapisink`

(Change the /dev/video device and the port number to add more webcams)
