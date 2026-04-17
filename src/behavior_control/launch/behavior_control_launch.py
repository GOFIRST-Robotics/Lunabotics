import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the path to your home directory
    home_dir = os.path.expanduser('~')
    
    # Manually construct the path to the workspace config folder
    behavior_control_config = os.path.join(
        home_dir, 'Lunabotics', 'config', 'behavior_control.yaml'
    )
    
    joy_config = os.path.join(
        home_dir, 'Lunabotics', 'config', 'joy_node.yaml'
    )

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
            name='behavior_control_node', # This MUST match the top-level key in your YAML
            parameters=[behavior_control_config], # Use the variable, not a string
            output='screen'
        ),
        Node(
            package="joy",
            executable="joy_node",
            parameters=[joy_config], # Use the absolute path variable
        )
    ])