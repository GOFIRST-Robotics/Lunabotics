import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rovr_control_launch_dir = PathJoinSubstitution([FindPackageShare('rovr_control'), ''])
    
    config = os.path.join(
        get_package_share_directory('behavior_control'),
        'config',
        'behavior_control.yaml'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([rovr_control_launch_dir, 'action_server_launch.py'])
        ),
        Node(
            package='behavior_control',
            executable='behavior_control_server',
            name='behavior_control_server',
            parameters=[config]
        )
    ])