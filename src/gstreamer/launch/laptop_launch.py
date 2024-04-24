from launch import LaunchDescription
from launch_ros.actions import Node


# Launches the joystick node and the gstreamer camera client node on the team laptop
def generate_launch_description():
    ld = LaunchDescription()

    joystick_node = Node(
        package="joy",
        executable="joy_node",
        parameters=["config/joy_node.yaml"],
    )

    GStreamer_node = Node(
        package="gstreamer",
        executable="server",
    )

    ld.add_action(joystick_node)
    ld.add_action(GStreamer_node)

    return ld
