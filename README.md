# Lunabotics

The official NASA Lunabotics GitHub repository for University of Minnesota Robotics.

[![Lint Check](https://github.com/GOFIRST-Robotics/Lunabotics/actions/workflows/linter_check.yml/badge.svg)](https://github.com/GOFIRST-Robotics/Lunabotics/actions/workflows/linter_check.yml) [![Trufflehog Scan](https://github.com/GOFIRST-Robotics/Lunabotics/actions/workflows/trufflehog_scan.yml/badge.svg)](https://github.com/GOFIRST-Robotics/Lunabotics/actions/workflows/trufflehog_scan.yml)

```mermaid
graph LR
    subgraph A[Operator Laptop]
        B[RQT Camera Frontend]
        J[joy_node]
    end
    subgraph C[Robot]
        subgraph F[Nvidia Jetson AGX Orin]
            G[motor_control]
            H[GStreamer NVENC AV1 Encoding]
            I[isaac_ros_nvblox]
            L[ros2socketcan_bridge]
            M[Nav2]
            N[Subsystem ROS 2 Nodes]
            O[rovr_control]
            P[ZED ROS 2 Wrapper]
        end
        D[Arduino Microcontroller]
        K[Limit Switches]
        E[VESC Motor Controllers]
    end
    K --> D
    O <-- Serial Bus --> D
    H -- WiFi Connection --> B
    L <-- CAN Bus --> E
    P --> I
    I -- Cost Map --> M
    M --> O
    J -- /joy --> O
    M -- /cmd_vel --> N
    G -- /CAN/can0/transmit --> L
    L -- /CAN/can0/receive --> G
    O -- /cmd_vel --> N
    O -- ROS 2 Services --> N
    N -- ROS 2 Services --> G
```

## How to Run Inside Docker Container

<details>
<summary>How to Run Inside the Dev Container on Windows/Mac</summary>
<br>
Open vscode and install the "Dev Containers" extension. Then, with vscode open, press ctrl+shift+p to open the vscode command palette and type "Clone Repository in Container Volume". Select the "Dev Containers: Clone Repository in Container Volume" option, then select "Clone a repository from GitHub in a Container Volume". Search for and select our Lunabotics repository (the repository named "Lunabotics"). If you are cloning the repository directly into the container volume, you do NOT need to clone the repo locally, it will be automatically cloned into the repo.
<br><br>

After opening the container, you can run the following command in the Command Palette (Ctrl + Shift + P) to build the project:
```
Tasks: Run Build Task
```

Or, if your machine does not have an Nvidia GPU or you haven't set it up with [container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html), run the following commands instead in the Command Palette (Ctrl + Shift + P):
```
Tasks: Configure Default Build Task
Build No GPU Tasks
```

Optionally, traditional "colcon build" commands can be run in the vscode terminal instead of using the Command Palette commands above.
</details>
<details>
<summary>Updating the Dev Container for Windows/Mac</summary>
<br>
If you ever need to rebuild the remote container image, first update the x86_64 and aarch64 images:

```
cd ~/Lunabotics/src/isaac_ros/isaac_ros_common/docker
docker build --build-arg="BASE_IMAGE=osrf/ros:humble-desktop" -f Dockerfile.user -t umnrobotics/devcontainer:x86_64.user .
cd ~/Lunabotics/docker
docker build --build-arg="BASE_IMAGE=umnrobotics/devcontainer:x86_64.user" -f Dockerfile.umn -t umnrobotics/devcontainer:x86_64.user.umn .
docker push umnrobotics/devcontainer:x86_64.user.umn

cd ~/Lunabotics/src/isaac_ros/isaac_ros_common/docker
docker build --build-arg="BASE_IMAGE=arm64v8/ros:humble" -f Dockerfile.user -t umnrobotics/devcontainer:arm64.user .
cd ~/Lunabotics/docker
docker build --build-arg="BASE_IMAGE=umnrobotics/devcontainer:arm64.user" -f Dockerfile.umn -t umnrobotics/devcontainer:arm64.user.umn .
docker push umnrobotics/devcontainer:arm64.user.umn
```

Then, run the following command with the devcontainer cli installed:
```
cd ~/Lunabotics
docker pull umnrobotics/devcontainer:x86_64.user.umn
docker pull umnrobotics/devcontainer:arm64.user.umn

docker manifest rm umnrobotics/devcontainer:latest
docker manifest create umnrobotics/devcontainer:latest --amend umnrobotics/devcontainer:arm64.user.umn --amend umnrobotics/devcontainer:x86_64.user.umn
docker manifest push umnrobotics/devcontainer:latest

docker buildx create --use
devcontainer build --push true --workspace-folder . --platform="linux/amd64,linux/arm64" --image-name "umnrobotics/ros:ros_devcontainer"
```
</details>

<details>
<summary>How to Run Inside the ISAAC ROS Container on Linux/Jetson</summary>
<br>
First, you will need to log in to Nvidia NGC and get an API Key here: https://org.ngc.nvidia.com/setup

Then install Nvidia ngc CLI and make sure it is present in path: https://org.ngc.nvidia.com/setup/installers/cli

Follow the instructions on the website to install and configure ngc.
    
Test the ngc installation by running `ngc` in a new terminal. If it doesn't work, try adding `echo "export PATH=\"\$PATH:$(pwd)/ngc-cli\"" >> ~/.bash_profile && source ~/.bash_profile` to your `~/.bashrc` file.

Then log in to nvcr with the following command:

```
docker login nvcr.io

Username: $oauthtoken
Password: <Your Key>
```

Install git-lfs with `sudo apt install git-lfs`

Run this command to build and enter the isaac ros container
```
./scripts/enter_isaac_ros_container.sh
```

</details>

## ROS 2 General Workspace Tips

When cloning this repository manually on Linux, you must run `git submodule update --init --recursive` inside the workspace folder to recursively pull all git submodules before building the project.

Use `colcon build --symlink-install` when building so that Python nodes do not need to be rebuilt every time.
Additionally, use these commands to avoid building unnecessary packages when testing:
`colcon build --symlink-install --packages-up-to <package name>`
or `colcon build --symlink-install --packages-select <package name>`

Make sure to `source install/setup.bash` in every new terminal.

Run `rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y --skip-keys "nvblox negotiated"` to install package dependencies.

Run `ros2 run <package_name> <node_name>` to run just the specified node.

Run `ros2 launch <package_name> <launch file name>` to launch a launch file.

Run `rm -rf build install log` to clean your workspace if you need to build completely from scratch.

Run `docker system prune -a --volumes` to remove all Docker images, containers, and volumes from your machine. This is useful when the ISAAC ROS container gets messed up and you want to rebuild the container from scratch.

To normalize line endings in git, use the command:
```
git config --global core.autocrlf true
```

## Gazebo Simulation

<details>
<summary>Gazebo Installation & Resources</summary>
<br>
  
Install Gazebo Fortress by running: `sudo apt-get install ros-humble-ros-gz`

More info [here](https://gazebosim.org/docs/garden/ros_installation). Remember that we are using ROS 2 Humble.

Instructions for building the ROS bridge (ros_gz) can be found [here](https://github.com/gazebosim/ros_gz/tree/humble#from-source).

Information about ROS types -> gazebo types can be found [here](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md)
</details>

<details>
<summary>Running the Gazebo Simulation</summary>
<br>
  
To run the Gazebo simulation:
```
colcon build --symlink-install --packages-up-to ros_gz_launch
source install/setup.bash
ros2 launch ros_gz_launch UCF_field.launch.py
```

Then to control the robot, you will need to run:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
in another terminal to control the robot with your keyboard.

Alternatively, you can run these nodes:
```
ros2 run joy joy_node --ros-args --params-file config/joy_node.yaml
ros2 run rovr_control main_control_node
```
to control the robot using a gamepad and our button bindings assigned in the main_control_node.
</details>

## Start the Joystick Node with params

```
ros2 run joy joy_node --ros-args --params-file config/joy_node.yaml
```

## Apriltag Detection Setup

Follow [this](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag/blob/main/docs/tutorial-usb-cam.md) tutorial to set up Apriltag detection on your machine.

## VESC CAN Bus References

[VESC CAN Status Frames Spreadsheet](https://github.com/codermonkey42/VESC_CAN)

[VESC 6 CAN Formats](https://vesc-project.com/sites/default/files/imce/u15301/VESC6_CAN_CommandsTelemetry.pdf)

[VESC Control with CAN](https://dongilc.gitbook.io/openrobot-inc/tutorials/control-with-can)

<details>
<summary>How to load the CAN modules at startup on Nvidia Jetson</summary>
<br>

Follow [these](https://docs.nvidia.com/jetson/archives/r36.4/DeveloperGuide/HR/ControllerAreaNetworkCan.html) instructions to enable CAN communication on Nvidia Jetson Orin.

1: Put the following in the `modules.conf` file in `/etc/modules-load.d/`

```
# Load the CAN bus kernel modules
can
can_raw
mttcan
#eof
```

2: Find the file `/etc/modprobe.d/denylist-mttcan.conf` and delete it if it exists (The filename might also be `/etc/modprobe.d/blacklist-mttcan.conf`)

3: Make a script called "can_startup.sh" in the `/root` directory of the system, with the following contents:
```
#! /usr/bin/sh

sudo ip link set can0 up type can bitrate 500000
sudo ip link set can1 up type can bitrate 500000
```

4: Run the command "sudo crontab -e" and add this line to the bottom of the file that appears:

```
@reboot sleep 5 && echo 'robot' | sudo -S sh can_startup.sh 2>&1 | logger -t mycmd
```

And that should work! If it doesn't and you need to read the log output of the crontab, use this command:

```
sudo grep -a 'mycmd' /var/log/syslog
```
</details>

## GStreamer References

[Accelerated GStreamer Guide](https://docs.nvidia.com/jetson/archives/r35.2.1/DeveloperGuide/text/SD/Multimedia/AcceleratedGstreamer.html)

<details>
<summary>Gstreamer Server/Client Instructions</summary>
<br>
To start gstreamer client make sure to add the deepstream layer to the docker layers 

To start the gstreamer client run the following commands:

```bash
colcon build --symlink-install --packages-up-to gstreamer
source install/setup.bash
rqt --force-discover
```

To start the gstreamer server run the following commands:

```bash
colcon build --symlink-install --packages-up-to gstreamer
source install/setup.bash
ros2 run gstreamer server_node
```
</details>

## Set static serial ports on the Jetson

Follow [these](https://msadowski.github.io/linux-static-port/) instructions.

## Install Nvidia Drivers / CUDA Toolkit on Ubuntu 22.04
Follow [these](https://developer.nvidia.com/cuda-12-6-0-download-archive?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=deb_local) instructions.

## Jetson External HDD Commands

```
sudo service docker stop
sudo mv /var/lib/docker /hd/docker
sudo ln -s /hd/docker /var/lib/docker # Create a symbolic link
sudo service docker start
```
