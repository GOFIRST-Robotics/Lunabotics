

To install ROS2 eloquent on  turtlebot using rpi 3B+:
use image 
https://cdimage.ubuntu.com/releases/18.04/release/
ubuntu-18.04.5-preinstalled-server-arm64+raspi3.img.xz

<strike>
https://releases.ubuntu-mate.org/archived/18.04/arm64/

Connect monitor and keyboard and use GUI to do language, keyboard, user, and wifi setup. Wait for it to login to the user that it creates and run some scripts after that happens.

Follow this to get ssh enabled:
https://askubuntu.com/questions/921258/how-to-enable-ssh-on-boot-in-ubuntu-mate
TLDR: run
```
sudo apt-get remove openssh-server
sudo apt-get install openssh-server
# I rebooted system after install
sudo systemctl enable ssh
```
</strike>

Install ROS2 according to these steps
https://docs.ros.org/en/eloquent/Installation/Linux-Install-Binary.html

run ```rosdep update --include-eol-distros``` at some point

Setup turtlebot starting on this page (make sure to select the "Dashing" tab at the top for ROS2 instructions): https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup

If you get colcon build errors, check memory usage with ```free -m ```. If it's full, look up how to increase the size of the swap file.
