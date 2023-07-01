import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    realsense_camera_node = ComposableNode(
        package="realsense2_camera",
        plugin="realsense2_camera::RealSenseNodeFactory",
        name="realsense2_camera",
        namespace="",
        parameters=[
            {
                "depth_module.profile": "1280x720x30",
                "rgb_camera.profile": "1920x1080x30",
            }
        ],
        remappings=[
            ("/color/image_raw", "/image"),
            ("/color/camera_info", "/camera_info"),
        ],
    )

    realsense_container = ComposableNodeContainer(
        package="rclcpp_components",
        name="realsense_container",
        namespace="",
        executable="component_container_mt",
        composable_node_descriptions=[realsense_camera_node],
        output="screen",
    )

    return launch.LaunchDescription([realsense_container])
