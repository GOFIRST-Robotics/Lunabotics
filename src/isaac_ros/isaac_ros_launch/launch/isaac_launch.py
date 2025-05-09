from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    # Launch Configurations
    setup_for_zed = LaunchConfiguration("setup_for_zed", default="True")
    use_nvblox = LaunchConfiguration("use_nvblox", default="True")
    zed_multicam = LaunchConfiguration("zed_multicam", default="False")
    use_apriltags = LaunchConfiguration("use_apriltags", default="True")
    setup_for_gazebo = LaunchConfiguration("setup_for_gazebo", default="False")

    # Launch Arguments
    run_rviz_arg = DeclareLaunchArgument("run_rviz_robot", default_value="True", description="Whether to start RVIZ")
    setup_for_zed_arg = DeclareLaunchArgument(
        "setup_for_zed",
        default_value="True",
        description="Whether to run from live zed data",
    )
    setup_for_gazebo_arg = DeclareLaunchArgument(
        "setup_for_gazebo",
        default_value="False",
        description="Whether to run in gazebo",
    )
    use_nvblox_arg = DeclareLaunchArgument(
        "use_nvblox",
        default_value="True",
        description="Whether to run nvblox",
    )
    use_nav2_arg = DeclareLaunchArgument(
        "use_nav2",
        default_value="True",
        description="Whether to run nav2",
    )
    record_svo_arg = DeclareLaunchArgument(
        "record_svo",
        default_value="False",
        description="Whether to record a ZED svo file",
    )
    zed_multicam_arg = DeclareLaunchArgument(
        "zed_multicam",
        default_value="False",
        description="Whether to use two ZED cameras",
    )
    use_apriltags_arg = DeclareLaunchArgument(
        "use_apriltags",
        default_value="True",
        description="Whether to run isaac ros apriltags",
    )

    global_frame = LaunchConfiguration("global_frame", default="odom")

    # Create a shared container to hold composable nodes
    # for speed ups through intra process communication.
    shared_container_name = "shared_nvblox_container"
    shared_container = Node(
        name=shared_container_name,
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_nvblox")),
    )

    # ZED
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("isaac_ros_launch"), "zed2i.launch.py"])]
        ),
        launch_arguments={
            "record_svo": LaunchConfiguration("record_svo"),
        }.items(),
        condition=IfCondition(PythonExpression([setup_for_zed, " and not ", zed_multicam])),
    )
    zed_multicam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("isaac_ros_launch"), "zed_multi_camera.launch.py"])]
        ),
        launch_arguments={
            "record_svo": LaunchConfiguration("record_svo"),
        }.items(),
        condition=IfCondition(PythonExpression([setup_for_zed, " and ", zed_multicam])),
    )

    # Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("ros_gz_launch"), "launch", "COSMIC_field.launch.py"])]
        ),
        condition=IfCondition(LaunchConfiguration("setup_for_gazebo")),
    )

    # Nvblox
    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("isaac_ros_launch"), "nvblox.launch.py"])]
        ),
        launch_arguments={
            "global_frame": global_frame,
            "setup_for_zed": LaunchConfiguration("setup_for_zed"),
            "setup_for_gazebo": LaunchConfiguration("setup_for_gazebo"),
            "attach_to_shared_component_container": "True",
            "component_container_name": shared_container_name,
        }.items(),
        condition=IfCondition(PythonExpression([use_nvblox, " and not ", zed_multicam])),
    )
    nvblox_multicam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("isaac_ros_launch"), "nvblox_multicam.launch.py"])]
        ),
        launch_arguments={
            "global_frame": global_frame,
            "setup_for_zed": LaunchConfiguration("setup_for_zed"),
            "setup_for_gazebo": LaunchConfiguration("setup_for_gazebo"),
            "attach_to_shared_component_container": "True",
            "component_container_name": shared_container_name,
        }.items(),
        condition=IfCondition(PythonExpression([use_nvblox, " and ", zed_multicam])),
    )

    # Rviz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("isaac_ros_launch"), "rviz.launch.py"])]),
        launch_arguments={
            "config_name": "zed_example.rviz",
            "global_frame": global_frame,
        }.items(),
        condition=IfCondition(LaunchConfiguration("run_rviz_robot")),
    )

    # Nav2 params
    nav2_param_file = PathJoinSubstitution(["config", "nav2_isaac_sim.yaml"])
    param_substitutions = {"global_frame": LaunchConfiguration("global_frame", default="odom")}
    configured_params = RewrittenYaml(
        source_file=nav2_param_file,
        root_key="",
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    # nav2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"])]
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_nav2")),
    )

    # apriltag launch
    apriltag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("apriltag"), "apriltag_launch.py"])]),
        condition=IfCondition(PythonExpression([use_apriltags, " and not ", setup_for_gazebo])),
    )
    # apriltag (gazebo) launch
    apriltag_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("apriltag"), "apriltag_gazebo_launch.py"])]
        ),
        condition=IfCondition(PythonExpression([use_apriltags, " and ", setup_for_gazebo])),
    )

    return LaunchDescription(
        [
            run_rviz_arg,
            setup_for_zed_arg,
            setup_for_gazebo_arg,
            record_svo_arg,
            use_nvblox_arg,
            use_nav2_arg,
            zed_multicam_arg,
            use_apriltags_arg,
            shared_container,
            nvblox_launch,
            nvblox_multicam_launch,
            nav2_launch,
            zed_launch,
            zed_multicam_launch,
            gazebo_launch,
            rviz_launch,
            apriltag_launch,
            apriltag_gazebo_launch,
        ]
    )
