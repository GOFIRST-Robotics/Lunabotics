from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

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
        parameters=["config/drivetrain_config.yaml", "config/motor_control.yaml"],
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
    )

    read_serial = Node(
        package="rovr_control",
        executable="read_serial",
        name="read_serial",
    )

    can_bus = Node(
        package="ros2socketcan_bridge",
        executable="ros2socketcan",
        name="ros2socketcan",
    )

    # Add our autonomous action servers here
    calibrate_field_coordinate_server = Node(
        package="rovr_control",
        executable="calibrate_field_coordinate_server",
        name="calibrate_field_coordinate_server",
    )
    auto_dig_server = Node(
        package="rovr_control",
        executable="auto_dig_server",
        name="auto_dig_server",
    )
    auto_offload_server = Node(
        package="rovr_control",
        executable="auto_offload_server",
        name="auto_offload_server",
    )

    ld.add_action(rovr_control)
    ld.add_action(motor_control)
    ld.add_action(drivetrain)
    ld.add_action(skimmer)
    ld.add_action(read_serial)
    ld.add_action(can_bus)
    ld.add_action(calibrate_field_coordinate_server)
    ld.add_action(auto_dig_server)
    ld.add_action(auto_offload_server)

    return ld
