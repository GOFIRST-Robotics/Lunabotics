from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    rovr_control = Node(
        package="rovr_control",
        executable="main_control_node",
    )

    motor_control = Node(package="motor_control", executable="motor_control_node")

    ld.add_action(rovr_control)
    ld.add_action(motor_control)

    return ld
