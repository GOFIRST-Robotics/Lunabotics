import os


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    bringup_dir = get_package_share_directory("isaac_ros_launch")

    # The CAN controllers need to be wired up and CAN modules enabled for this ROS 2 node to work
    ros2socketcan_bridge = Node(
        package="ros2socketcan_bridge",
        executable="ros2socketcan",
        name="ros2socketcan",
        output="screen",
        emulate_tty=True,
    )

    # The Arduino needs to be connected to the USB port for this ROS 2 node to work
    read_serial = Node(
        package="rovr_control",
        executable="read_serial",
        name="read_serial_node",
        output="screen",
        emulate_tty=True,
    )

    isaac_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, "isaac_launch.py")),
        launch_arguments={
            "setup_for_zed": "True",
            "setup_for_gazebo": "False",
            "use_nvblox": "True",
        }.items(),
    )

    # This ROS 2 node implements the main control logic for the robot
    rovr_control = Node(
        package="rovr_control",
        executable="main_control_node",
        name="main_control_node",
        parameters=["config/rovr_control.yaml"],
        output="screen",
        emulate_tty=True,
    )
    # This ROS 2 node implements motor control code for our VESCs
    motor_control = Node(
        package="motor_control",
        executable="motor_control_node",
        name="motor_control_node",
        parameters=["config/motor_control.yaml"],
        output="screen",
        emulate_tty=True,
    )

    # This is the ROS 2 node for the drivetrain subsystem
    drivetrain = Node(
        package="drivetrain",
        executable="drivetrain_node",
        name="drivetrain_node",
        parameters=["config/drivetrain_config.yaml", "config/motor_control.yaml"],
        output="screen",
        emulate_tty=True,
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

    # This is the ROS 2 wrapper for the Intel Realsense d435 camera
    realsense = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="camera",
        namespace="skimmer",
        # remappings=[
        #     ('/camera/realsense2_camera/depth/image_rect_raw', '/depth/image_rect_raw'),
        #     ('/camera/camera/depth/image_rect_raw', '/depth/image_rect_raw'),
        #     ('/camera/depth/image_rect_raw', '/depth/image_rect_raw'),
        #     ('/depth/image_rect_raw', '/depth/image_rect_raw'),
        # ]
    )

    # This node uses an Intel Realsense d435 camera to check the material load on the skimmer
    check_load = Node(
        package="skimmer",
        executable="ros_check_load",
        name="ros_check_load",
    )

    # Add all of the actions to the launch description
    ld.add_action(ros2socketcan_bridge)
    ld.add_action(read_serial)
    ld.add_action(rovr_control)
    ld.add_action(motor_control)
    ld.add_action(drivetrain)
    ld.add_action(skimmer)
    ld.add_action(realsense)
    ld.add_action(check_load)
    ld.add_action(isaac_launch)
    # Return the launch description
    return ld
