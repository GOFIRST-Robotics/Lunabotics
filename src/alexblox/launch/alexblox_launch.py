import launch
from launch_ros.actions import Node


def generate_launch_description():
    alexblox_node = Node(
        package="alexblox",
        executable="costmapGenerator",
        name="costmap",
        parameters=["config/costmap_config.yaml"],
    )

    return launch.LaunchDescription([alexblox_node])
