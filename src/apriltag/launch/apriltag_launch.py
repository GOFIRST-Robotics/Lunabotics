from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    """need to swap usb cam to zed when we test with that, but this might work for now"""
    cameraOn = Node(
        package="isaac_ros_apriltag",
        executable="isaac_ros_apriltag_usb_cam.launch.py",
        name="isaac_ros_apriltag_usb_cam",
    )
    
    updateMap = Node(
        package="apriltag",
        executable="apriltag",
        name="apriltag",
    )
    
    ld.add_action(cameraOn)
    ld.add_action(updateMap)

    return ld