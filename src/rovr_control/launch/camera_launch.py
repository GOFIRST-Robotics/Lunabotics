from launch import LaunchDescription
from launch_ros.actions import Node


def generate_camera_nodes(
    camera_name, video_device, pixel_format="yuyv", resolution=(320, 240)
):
    return [
        # Camera driver node
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            namespace=camera_name,
            name="camera",
            parameters=[
                {"video_device": video_device},
                {"image_width": resolution[0]},
                {"image_height": resolution[1]},
                {"pixel_format": pixel_format},
            ],
            remappings=[("/image_raw", "image_raw"), ("/camera_info", "camera_info")],
            # scoped to namespace
        ),
        # Compressed republisher
        # Node(
        #     package="image_transport",
        #     executable="republish",
        #     namespace=camera_name,
        #     name="republish_compressed",
        #     arguments=["raw", "compressed"],
        #     remappings=[("in", f"{video_device}/image_raw"), ("out", "image_raw/compressed")],
        # ),
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_camera_nodes("right", "/dev/video6", "mjpeg2rgb")
        + generate_camera_nodes("left", "/dev/video8", "mjpeg2rgb")
        + generate_camera_nodes("back", "/dev/video0", "mjpeg2rgb")
        + generate_camera_nodes("digger", "/dev/video4")
        + generate_camera_nodes("dumper", "/dev/video5")
        # + generate_camera_nodes("front", "/dev/video0")
    )
