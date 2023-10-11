from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    realsense = Node(
        package="realsense2_camera",
        executable="rs_launch.py",
        name="realsense2_camera",
    )
    
    check_load = Node(
        package="conveyor",
        executable="ros_check_load",
        name="ros_check_load",
    )
    
    ld.add_action(realsense)
    ld.add_action(check_load)

    return ld