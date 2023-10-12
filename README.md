# Lunabotics-2024

The official NASA Lunabotics 2024 repository for University of Minnesota Robotics.

![Control-Flow Diagram](assets/NASA-Lunabotics-Software-Diagram.png)

## How to Run Inside Docker Container

Open this repository in vscode then press ctrl+shift+p and type "Remote-Containers: Reopen in Container".
Just press "from dockerfile" and then it will build the container and run it.

After opening, run the following commands in the terminal:

```
colcon build --symlink-install
source install/setup.sh
```

If your machine does not have an Nvidia GPU, build using this command instead:

```
colcon build --symlink-install --packages-skip-regex zed*
```

If you need to rebuild the remote container image, uncomment the sections in devcontainer that reference remote, then run the following command with the devcontainer cli installed:

```
devcontainer build --push true --workspace-folder . --platform="linux/amd64,linux/arm64" --image-name "umnrobotics/ros"
```

## ROS 2 General Workspace Tips

Make sure to `source install/setup.bash` in every new terminal.

Run `rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y` to install package dependencies.

Run `rm -r build install log` to clean your workspace.

To configure Sonarlint for C++ linting, run the following command:
```
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```
Then, point Sonarlint to the `compile_commands.json` file that is created in your `build` directory.

To normalize line endings in git, use the command:
```
git config --global core.autocrlf true
```

## Start the Joystick Node with params

```
ros2 run joy joy_node --ros-args --params-file config/joy_node.yaml
```

## Apriltag Detection Setup

Follow [this](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag/blob/main/docs/tutorial-usb-cam.md) tutorial to set up Apriltag detection on your machine.

## Useful Gstreamer Commands

Start Gstreamer AV1 Encoding (On Nvidia Jetson AGX Orin): 

```
gst-launch-1.0 v4l2src device=/dev/video0 ! "video/x-raw,width=640,height=480,framerate=15/1" ! nvvidconv ! "video/x-raw(memory:NVMM),format=NV12" ! nvv4l2av1enc bitrate=200000 ! "video/x-av1" ! udpsink host=127.0.0.1 port=5000
```

Start Gstreamer AV1 Decoding (On Nvidia Jetson AGX Orin): 

```
gst-launch-1.0 udpsrc port=5000 ! "video/x-av1,width=640,height=480,framerate=15/1" ! queue ! nvv4l2decoder ! nv3dsink
```

Start Gstreamer AV1 Decoding (On Ubuntu Laptop w/ Docker runtime need nvcr login): 

```
xhost +
docker run -it --rm --net=host --gpus all -e DISPLAY=$DISPLAY --device /dev/snd -v /tmp/.X11-unix/:/tmp/.X11-unix nvcr.io/nvidia/deepstream:6.3-triton-multiarch 
gst-launch-1.0 udpsrc port=5000 ! "video/x-av1,width=640,height=480,framerate=15/1" ! queue ! nvv4l2decoder ! nveglglessink
```

Start Gstreamer H.264 Encoding (On Nvidia Jetson Orin Nano): 

```
gst-launch-1.0 v4l2src device=/dev/video0 ! "video/x-raw,width=640,height=480,framerate=15/1" ! nvvidconv ! "video/x-raw,format=I420" ! x264enc bitrate=300 tune=zerolatency speed-preset=ultrafast ! "video/x-h264,stream-format=byte-stream" ! h264parse ! rtph264pay ! udpsink host=127.0.0.1 port=5000
```

Start Gstreamer H.264 Decoding (On Nvidia Jetson Orin Nano): 

```
gst-launch-1.0 udpsrc port=5000 ! "application/x-rtp,payload=96" ! rtph264depay ! h264parse ! avdec_h264 ! nvvidconv ! xvimagesink
```

Start Gstreamer H.264 Decoding (On Ubuntu Laptop): 

```
gst-launch-1.0 udpsrc port=5000 ! application/x-rtp, encoding-name=H264, payload=96 ! rtph264depay ! h264parse ! nvh264dec ! videoflip method=vertical-flip ! xvimagesink sync=false
```

(Change the /dev/video device to add more webcams, and the port number to stream multiple webcams at once)

## VESC CAN Bus Resources/References

[VESC CAN Status Frames Spreadsheet](https://github.com/codermonkey42/VESC_CAN)

[VESC 6 CAN Formats](https://vesc-project.com/sites/default/files/imce/u15301/VESC6_CAN_CommandsTelemetry.pdf)

[VESC Control with CAN](https://dongilc.gitbook.io/openrobot-inc/tutorials/control-with-can)

To load the CAN modules at startup do the following:

1: Put the following in a .conf file in /modules-load.d/

```
#Setting up the CAN bus 
can
can_raw
mttcan
#eof
```

2: Find the file /etc/modprobe.d/denylist-mttcan.conf and either delete it or comment out the one line in it (The filename might be .../blacklist-mttcan.conf)

3: Make a script called "can_startup.sh" in the root directory for the system, with the following contents:
```
#! /usr/bin/sh

sudo ip link set can0 up type can bitrate 500000
sudo ip link set can1 up type can bitrate 500000
```

4: Run the command "sudo crontab -e" and put this line in the file that appears:

```
@reboot sleep 5 && echo 'robot' | sudo -S sh /can_startup.sh 2>&1 | logger -t mycmd
```

And that should work. If it doesn't and you need to read the output of the crontab, use this command:

```
sudo grep 'mycmd' /var/log/syslog
```

## Jetson External HDD Commands

```
sudo service docker stop
sudo mv /var/lib/docker /hd/docker
sudo ln -s /hd/docker /var/lib/docker # Create a symbolic link
sudo service docker start
```

## GStreamer Resources/References

[Accelerated GStreamer Guide](https://docs.nvidia.com/jetson/archives/r35.2.1/DeveloperGuide/text/SD/Multimedia/AcceleratedGstreamer.html)
