import os

from sympy import Ge

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory("isaac_ros_launch")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    launch_desc = LaunchDescription()
    # Launch Arguments
    run_rviz_arg = DeclareLaunchArgument(
        "run_rviz", default_value="True", description="Whether to start RVIZ"
    )
    setup_for_zed_arg = DeclareLaunchArgument(
        "setup_for_zed",
        default_value="False",
        description="Whether to run from live zed data",
    )
    setup_for_gazebo_arg = DeclareLaunchArgument(
        "setup_for_gazebo",
        default_value="True",
        description="Whether to run in gazebo",
    )

    launch_desc.add_action(run_rviz_arg)
    launch_desc.add_action(setup_for_zed_arg)
    launch_desc.add_action(setup_for_gazebo_arg)
    

    global_frame = LaunchConfiguration("global_frame", default="odom")
    # setup_for_zed = IfCondition(LaunchConfiguration("setup_for_gazebo", default="False"))
    # setup_for_gazebo = IfCondition(LaunchConfiguration("setup_for_gazebo", default="False"))

    # Create a shared container to hold composable nodes
    # for speed ups through intra process communication.
    shared_container_name = "shared_nvblox_container"
    shared_container = Node(
        name=shared_container_name,
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
    )
    launch_desc.add_action(shared_container)
    # Nvblox
    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(bringup_dir, "nvblox.launch.py")]),
        launch_arguments={
            "global_frame": global_frame,
            "setup_for_zed": LaunchConfiguration("setup_for_zed"),
            "setup_for_gazebo": LaunchConfiguration("setup_for_gazebo"),
            "attach_to_shared_component_container": "True",
            "component_container_name": shared_container_name,
        }.items(),
    )
    launch_desc.add_action(nvblox_launch)
    # Nav2
    nav2_param_file = os.path.join("config", "nav2_isaac_sim.yaml")
    param_substitutions = {
        "global_frame": LaunchConfiguration("global_frame", default="odom")
    }
    configured_params = RewrittenYaml(
        source_file=nav2_param_file,
        root_key="",
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    # nav2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "False",
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )
    launch_desc.add_action(nav2_launch)

    # ZED
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(bringup_dir, "zed2i.launch.py")]),
        launch_arguments={
            "attach_to_shared_component_container": "True",
            "component_container_name": shared_container_name,
        }.items(),
        condition=IfCondition(LaunchConfiguration("setup_for_zed"))
    )
    launch_desc.add_action(zed_launch)
    
    odom_tf_publisher = Node(
        package="isaac_ros_launch",
        executable="odom_tf_publisher",
        name="odom_tf_publisher",
        condition=IfCondition(LaunchConfiguration("setup_for_gazebo"))
    )
    launch_desc.add_action(odom_tf_publisher)

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("ros_gz_launch"), "launch", "UCF_field.launch.py")]),
        condition=IfCondition(LaunchConfiguration("setup_for_gazebo"))
    )
    launch_desc.add_action(gazebo_launch)
    # Rviz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    bringup_dir,
                    "rviz.launch.py",
                )
            ]
        ),
        launch_arguments={
            "config_name": "zed_example.rviz",
            "global_frame": global_frame,
        }.items(),
        condition=IfCondition(LaunchConfiguration("run_rviz")),
    )
    launch_desc.add_action(rviz_launch)

    return launch_desc
