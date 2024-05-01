import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    main_bringup_dir = get_package_share_directory("rovr_control")
    isaac_bringup_dir = get_package_share_directory("isaac_ros_launch")
    check_load_bringup_dir = get_package_share_directory("skimmer")

    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(main_bringup_dir, "main_launch_no_joysticks.py")),
    )

    isaac_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(isaac_bringup_dir, "isaac_launch.py")),
        launch_arguments={
            "setup_for_zed": "True",
            "setup_for_gazebo": "False",
            "use_nvblox": "True",
        }.items(),
    )

    check_load_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(check_load_bringup_dir, "check_load_launch.py")),
    )

    # This is the ROS 2 node for the skimmer subsystem
    skimmer = Node(
        package="skimmer",
        executable="skimmer_node",
        name="skimmer_node",
        parameters=["config/motor_control.yaml"],
        output="screen",
        emulate_tty=True,
    )

    # Add all of the actions to the launch description
    ld.add_action(main_launch)
    ld.add_action(skimmer)
    ld.add_action(check_load_launch)
    ld.add_action(isaac_launch)
    # Return the launch description
    return ld
