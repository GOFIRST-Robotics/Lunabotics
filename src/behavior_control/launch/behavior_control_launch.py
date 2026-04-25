import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('rovr_control'), 
                '', # Added 'launch' subfolder if applicable
                'action_server_launch.py'
            ])
        ),
        Node(
            package='behavior_control',
            executable='behavior_control_node',
            name='behavior_control_node',
            parameters=["config/behavior_control.yaml"],
            output='screen'
        ),
        Node(
            package="joy",
            executable="joy_node",
            parameters=["config/joy_node.yaml"],
            output='screen'
        ),
        Node(
            package="rovr_control",
            executable="stream_deck_node",
            output='screen'
        )  
    ])