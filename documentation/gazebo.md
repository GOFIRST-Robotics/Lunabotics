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
ros2 run joy joy_node
ros2 run rovr_control main_control_node
```
to control the robot using a gamepad and our button bindings assigned in the main_control_node.
</details>
