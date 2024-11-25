# How to Set up Our Development Container on Various OS's
To have the best results you will need a Linux computer with a Nvidia GPU. ROS2 runs natively on the linux kernal and we use nvidia closed source packages. It is possible to run our codebase on windows with great difficultly. It is not possible to run all of our codebase on a computer without a Nvidia GPU.


## How to Run Inside the Dev Container on Windows/Mac 
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

### Updating the Dev Container for Windows/Mac
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

## How to Run Inside the ISAAC ROS Container on Linux/Jetson
This is intruction on how to install all the needed libaries and SDKs to run the container on a computer that is running linux and has a Nvidia GPU.

In all you will need [NGC](#installing-ngc), [Git-lfs](#installing-git-lfs), CUDA SDK (and Nvidia GPU drivers),
### Installing NGC
First, you will need to log in to Nvidia NGC and get an API Key [here](https://org.ngc.nvidia.com/setup){:target="_blank"}. You will need to click on "Guest Account" in the top right and log in/create account.

Then download Nvidia ngc CLI from this [link](https://org.ngc.nvidia.com/setup/installers/cli){:target="_blank"}. Make sure to select the correct OS and architecture type for your computer (if you dont know your architecture it is probably AMD64)
and then click the green "Download CLI" button in the top right. 
The download will be zip file with name either ``ngccli_linux.zip`` or ``ngccli_arm64.zip``.

I would recommend moving the downloaded zip file into your path (~) directory before continuing by
```
cd ~/Downloads/
mv ngccli_xxx.zip ~/
cd ~
```

You can follow the instructions on the website to install and configure ngc, but the main commands you need to run are 
```
wget --content-disposition https://api.ngc.nvidia.com/v2/resources/nvidia/ngc-apps/ngc_cli/versions/3.53.0/files/ngccli_linux.zip -O ngccli_linux.zip && unzip ngccli_linux.zip
chmod u+x ngc-cli/ngc
echo "export PATH=\"\$PATH:$(pwd)/ngc-cli\"" >> ~/.bash_profile && source ~/.bash_profile
```
Next you need to configure ngc with ``ngc config set`` and inputting the API key when prompted. The other values dont matter and you should be able to just hit enter to get the default, 
the only one that will need input is the org where you will need to copy the name that is listed in the choices.

Test the ngc installation by running `ngc` in a new terminal. If it doesn't work, try adding `source ~/.bash_profile` to your `~/.bashrc` file. Just open the ``.bashrc`` file in your favorite text editor and paste the source at the bottom.

Then log in to nvcr with the following command:
```
docker login nvcr.io
```
When prompted enter ``$oauthtoken`` (you can paste that exactly) for your Username
and ``<Your API key>`` (replace with same API key that inputted durig ngc config set up) for your Password.

### Installing Git-LFS
<br>
Install git-lfs with `sudo apt install git-lfs`

Run this command to build and enter the isaac ros container
```
./scripts/enter_isaac_ros_container.sh
```

</details>
