import os

# from launch_ros.actions import Node  # NOTE: Uncomment this code block if we get the gstreamer server working
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    main_bringup_dir = get_package_share_directory("rovr_control")
    isaac_bringup_dir = get_package_share_directory("isaac_ros_launch")
    # check_load_bringup_dir = get_package_share_directory("skimmer") # NOTE: Uncomment this line if we decide to actually use ros_check_load

    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(main_bringup_dir, "main_launch_no_joysticks.py")),
    )

    isaac_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(isaac_bringup_dir, "isaac_launch.py")),
        launch_arguments={
            "setup_for_zed": "True",
            "setup_for_gazebo": "False",
            "use_nvblox": "True",
            "run_rviz": "False",  # We don't need to run RViz during matches
            "record_svo": "True",  # Record match data to an SVO file
        }.items(),
    )

    # NOTE: Uncomment this code block if we decide to actually use ros_check_load
    # check_load_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(check_load_bringup_dir, "check_load_launch.py")),
    # )

    # NOTE: Uncomment this code block if we get the gstreamer server working
    # gstreamer_server = Node(
    #     package="gstreamer",
    #     executable="server_node",
    #     name="gstreamer_server_node",
    #     output="screen",
    #     emulate_tty=True,
    # )

    # Add all of the actions to the launch description
    ld.add_action(main_launch)
    # ld.add_action(check_load_launch) # NOTE: Uncomment this line if we decide to actually use ros_check_load
    # ld.add_action(gstreamer_server)  # NOTE: Uncomment this line if we get the gstreamer server working
    ld.add_action(isaac_launch)
    # Return the launch description
    return ld
