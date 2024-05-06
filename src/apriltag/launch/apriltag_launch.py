from sympy import use
import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from launch_ros.actions import LoadComposableNodes

def generate_launch_description():

    attach_to_shared_component_container_arg = LaunchConfiguration(
        "attach_to_shared_component_container", default="False"
    )

    component_container_name_arg = LaunchConfiguration(
        "component_container_name", default="apriltag_container"
    )
    
    apriltag_container = ComposableNodeContainer(
        package="rclcpp_components",
        name=LaunchConfiguration("component_container_name"),
        namespace="",
        executable="component_container_mt",
        output="screen",
        condition=UnlessCondition(attach_to_shared_component_container_arg),
    )

    load_composable_nodes = LoadComposableNodes(
        target_container=component_container_name_arg,
        composable_node_descriptions=[
            ComposableNode(
                package="isaac_ros_apriltag",
                plugin="nvidia::isaac_ros::apriltag::AprilTagNode",
                name="isaac_ros_apriltag",
                namespace="",
                remappings=[
                    ("image", "zed_node/left/image_rect_color_rgb"),
                    ("camera_info", "zed_node/left/camera_info"),
                ],
            ),
            ComposableNode(
                package="isaac_ros_image_proc",
                plugin="nvidia::isaac_ros::image_proc::ImageFormatConverterNode",
                name="image_format_node_left",
                parameters=[
                    {
                        "encoding_desired": "rgb8",
                    }
                ],
                remappings=[
                    ("image_raw", "zed_node/left/image_rect_color"),
                    ("image", "zed_node/left/image_rect_color_rgb"),
                ],
            ),
        ],
    )

    tag_reader = Node(
        package="apriltag",
        executable="apriltag_node",
        name="apriltag_node",
        parameters=["config/rovr_control.yaml"],
    )

    # Add nodes and containers to LaunchDescription
    return launch.LaunchDescription(
        [
            tag_reader,
            apriltag_container,
            load_composable_nodes
        ]
    )
