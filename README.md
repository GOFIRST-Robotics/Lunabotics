# LUNABOTICS-2023

The official NASA RMC 2023 repository for LUNABOTICS team, a segment of University of Minnesota Robotics.

We started developing on the Nvidia Jetson TX2 (running Ubuntu 18.04 and ROS 2 Eloquent) but eventually switched over to the newer Nvidia Jetson Orin Nano (running Ubuntu 20.04 and ROS 2 Foxy)

## How to Run Inside Docker Container

Open this repository in vscode then run ctrl-shift-p and type "Remote-Containers: Reopen in Container".
Just press "from dockerfile" and then it will build the container and run it.

When open, run the following commands in the terminal:

```
source /opt/ros/foxy/setup.sh
colcon build
source install/setup.sh
```

## ROS 2 General Workspace Tips

Make sure to `source install/setup.bash` in every new terminal.

Run `rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y` to install package dependencies.

Run `rm -r build install log` to clean your workspace.

## Joystick Node

```
ros2 run joy joy_node --ros-args --params-file config/joy_node.yaml
```

## NavX Node

```
ros2 run navx navx_node
```

## EKF Node

Start the navX Node first with the command above, and then 
run the script in /scripts that starts the static transform publisher from base_link to imu_link with the command:

```
./imu_link_transform_publisher.sh
```

Start the EKF node with ROS parameters: 

```
ros2 run robot_localization ekf_node ekf_filter_node --ros-args --params-file config/ekf.yaml
```

## Intel RealSense Camera Setup

Follow the instructions outlined [here](https://github.com/IntelRealSense/realsense-ros#installation) to set up the RealSense depth camera on your machine.

## Apriltag Detection Setup

Follow [this](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag/blob/main/docs/tutorial-usb-cam.md) tutorial to set up Apriltag detection on your machine.

## Gstreamer Commands

Start Gstreamer H.265 Encoding (On Nvidia Jetson TX2): 

```
gst-launch-1.0 v4l2src device=/dev/video0 ! "video/x-raw,width=640,height=480,framerate=30/1" ! nvvidconv ! "video/x-raw(memory:NVMM),format=I420" ! omxh265enc bitrate=200000 ! "video/x-h265,stream-format=byte-stream" ! h265parse ! rtph265pay ! udpsink host=127.0.0.1 port=5000
```

Start Gstreamer H.265 Decoding (On Nvidia Jetson TX2): 

```
gst-launch-1.0 udpsrc port=5000 ! "application/x-rtp,payload=96" ! rtph265depay ! h265parse ! omxh265dec ! nvvidconv ! xvimagesink
```

Start Gstreamer H.264 Encoding (On Nvidia Jetson Orin Nano): 

```
gst-launch-1.0 v4l2src device=/dev/video0 ! "video/x-raw,width=640,height=480,framerate=15/1" ! nvvidconv ! "video/x-raw,format=I420" ! x264enc bitrate=300 tune=zerolatency speed-preset=ultrafast ! "video/x-h264,stream-format=byte-stream" ! h264parse ! rtph264pay ! udpsink host=127.0.0.1 port=5000
```

Start Gstreamer H.264 Decoding (On Nvidia Jetson Orin Nano): 

```
gst-launch-1.0 udpsrc port=5000 ! "application/x-rtp,payload=96" ! rtph264depay ! h264parse ! avdec_h264 ! nvvidconv ! xvimagesink
```

Start Gstreamer H.265 Decoding (On Ubuntu Laptop): 

```
gst-launch-1.0 udpsrc port=5000 ! application/x-rtp, encoding-name=H265, payload=96 ! rtph265depay ! h265parse ! nvh265dec ! xvimagesink sync=false
```

Start Gstreamer H.264 Decoding (On Ubuntu Laptop): 

```
gst-launch-1.0 udpsrc port=5000 ! application/x-rtp, encoding-name=H264, payload=96 ! rtph264depay ! h264parse ! nvh264dec ! videoflip method=vertical-flip ! xvimagesink sync=false
```

(Change the /dev/video device to add more webcams, and the port number to stream multiple webcams at once)

## Useful Resources/References

Configuring the Nvidia Jetson TX2 for CAN Communication: 

1) https://www.mathworks.com/help/supportpkg/nvidia/ug/jetson-can-bus-traffic-sign-detection.html

2) https://forums.developer.nvidia.com/t/how-to-use-can-on-jetson-tx2/54125

[VESC CAN Status Frames Spreadsheet](https://github.com/codermonkey42/VESC_CAN)
