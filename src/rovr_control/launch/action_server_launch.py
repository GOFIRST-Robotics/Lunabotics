from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rovr_control',
            executable='auto_dig_nav_offload_server',
        ),
        Node(
            package='rovr_control',
            executable='auto_dig_server',
        ),
        Node(
            package='rovr_control',
            executable='auto_offload_server',
        ),
        Node(
            package='rovr_control',
            executable='calibrate_field_coordinate_server',
        ),
        Node(
            package='rovr_control',
            executable='dig_location_server',
        )
    ])