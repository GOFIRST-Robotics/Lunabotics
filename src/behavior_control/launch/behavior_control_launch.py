from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    rovr_control_launch_dir = PathJoinSubstitution([FindPackageShare('rovr_control'), ''])
    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([rovr_control_launch_dir, 'action_server_launch.py'])
        ),
        Node(
            package='behavior_control',
            executable='behavior_control_server',
        )
    ])