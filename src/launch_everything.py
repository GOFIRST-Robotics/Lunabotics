import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

import launch
from launch.substitutions import Command
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
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
    ld = LaunchDescription()

    motor_control = Node(
        package="motor_control",
        executable="motor_control_node",
        name="motor_control_node",
        output="screen",
        emulate_tty=True,
    )
    
    ## New node, might be cause of errors
    ros2socketcan_bridge = Node(
        package="ros2socketcan_bridge",
        executable="ros2socketcan",
        name="ros2socketcan",
        output="screen",
        emulate_tty=True,
    )

    ## New node, might be cause of errors
    read_serial = Node(
        package="read_serial",
        executable="read_serial",
        name="read_serial_node",
        output="screen",
        emulate_tty=True,
    )

    tag_reader = Node(
        package='apriltag',
        executable='apriltag',
        name='apriltag',
    )

    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        namespace='',
        remappings=[
            ('image', 'zed_node/left/image_rect_color_rgb'),
            ('camera_info', 'zed_node/left/camera_info')
        ]
    )

    image_format_converter_node_left = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='image_format_node_left',
        parameters=[{
                'encoding_desired': 'rgb8',
        }],
        remappings=[
            ('image_raw', 'zed_node/left/image_rect_color'),
            ('image', 'zed_node/left/image_rect_color_rgb')]
    )

    apriltag_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='apriltag_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            apriltag_node,
            image_format_converter_node_left
        ],
        output='screen'
    )

    # The zed camera mode name. zed, zed2, zed2i, zedm, zedx or zedxm
    camera_model = 'zed2i'

    # URDF/xacro file to be loaded by the Robot State Publisher node
    xacro_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'urdf', 'zed_descr.urdf.xacro'
    )

    # ZED Configurations to be loaded by ZED Node
    config_common = os.path.join(
        get_package_share_directory('isaac_ros_apriltag'),
        'config',
        'zed.yaml'
    )

    config_camera = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        camera_model + '.yaml'
    )

    # Robot State Publisher node
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='zed_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(
                [
                    'xacro', ' ', xacro_path, ' ',
                    'camera_name:=', camera_model, ' ',
                    'camera_model:=', camera_model
                ])
        }]
    )


    # ZED node using manual composition
    zed_node = Node(
        package='zed_wrapper',
        executable='zed_wrapper',
        output='screen',
        parameters=[
            config_common,  # Common parameters
            config_camera,  # Camera related parameters
        ]
    )
    

    bringup_dir = get_package_share_directory("isaac_ros_launch")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    # Launch Arguments
    run_rviz_arg = DeclareLaunchArgument(
        "run_rviz", default_value="True", description="Whether to start RVIZ"
    )
    from_bag_arg = DeclareLaunchArgument(
        "from_bag",
        default_value="False",
        description="Whether to run from a bag or live zed data",
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
    )

    # ZED
    # Note(remos): This was only tested with a ZED2 camera so far.
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(bringup_dir, "zed2i.launch.py")]),
        launch_arguments={
            "attach_to_shared_component_container": "True",
            "component_container_name": shared_container_name,
            "from_bag": LaunchConfiguration("from_bag"),
        }.items(),
    )

    # Nvblox
    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(bringup_dir, "nvblox.launch.py")]),
        launch_arguments={
            "global_frame": global_frame,
            "setup_for_zed": "True",
            "attach_to_shared_component_container": "True",
            "component_container_name": shared_container_name,
        }.items(),
    )

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


    rovr_control = Node(
        package="rovr_control",
        executable="main_control_node",
        name="main_control_node",
        parameters=["config/rovr_control.yaml"],
        output="screen",
        emulate_tty=True,
    )

    motor_control = Node(
        package="motor_control",
        executable="motor_control_node",
        name="motor_control_node",
        parameters=["config/motor_control.yaml"],
        output="screen",
        emulate_tty=True,
    )

    drivetrain = Node(
        package="drivetrain",
        executable="drivetrain_node",
        name="drivetrain_node",
        parameters=["config/drivetrain_config.yaml", "config/motor_control.yaml"],
        output="screen",
        emulate_tty=True,
    )
    skimmer = Node(
        package="skimmer",
        executable="skimmer_node",
        name="skimmer_node",
        parameters=["config/motor_control.yaml"],
        output="screen",
        emulate_tty=True,
    )

    realsense = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="camera",
        namespace="skimmer"
        # remappings=[
        #     ('/camera/realsense2_camera/depth/image_rect_raw', '/depth/image_rect_raw'),
        #     ('/camera/camera/depth/image_rect_raw', '/depth/image_rect_raw'),
        #     ('/camera/depth/image_rect_raw', '/depth/image_rect_raw'),
        #     ('/depth/image_rect_raw', '/depth/image_rect_raw'),
        # ]
    )
    
    joystick_node = Node(
        package="joy",
        executable="joy_node",
        parameters=["config/joy_node.yaml"],
    )
    
    check_load = Node(
        package="skimmer",
        executable="ros_check_load",
        name="ros_check_load",
    )

    ld.add_action(motor_control)
    ld.add_action(read_serial)
    ld.add_action(tag_reader)
    ld.add_action(apriltag_container)
    ld.add_action(rsp_node)
    ld.add_action(zed_node)
    ld.add_action(rovr_control)
    ld.add_action(motor_control)
    ld.add_action(joystick_node)
    ld.add_action(drivetrain)
    ld.add_action(skimmer)
    ld.add_action(realsense)
    ld.add_action(check_load)
    ld.add_action(run_rviz_arg)
    ld.add_action(from_bag_arg)
    ld.add_action(shared_container)
    ld.add_action(zed_launch)
    ld.add_action(nvblox_launch)
    ld.add_action(rviz_launch)
    ld.add_action(nav2_launch)
    return ld
