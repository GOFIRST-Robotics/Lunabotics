from launch import LaunchDescription
from launch_ros.actions import Node


def generate_camera_nodes(camera_name, video_device, resolution=(640, 480), quality=60):
    return [
        # Camera driver node
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            namespace=camera_name,
            name="camera",
            parameters=[
                {"video_device": video_device},
                {"image_size": [resolution[0], resolution[1]]},
                {"pixel_format": "YUYV"},
                {".image_raw.jpeg_quality": quality},
            ],
            remappings=[("/image_raw", "image_raw"), ("/camera_info", "camera_info")],  # scoped to namespace
        ),
        # Compressed republisher
        Node(
            package="image_transport",
            executable="republish",
            namespace=camera_name,
            name="republish_compressed",
            arguments=["raw", "compressed"],
            remappings=[("in", f"{video_device}/image_raw"), ("out", "image_raw/compressed")],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_camera_nodes("digger", "/dev/video0")
        + generate_camera_nodes("left", "/dev/video7")
        + generate_camera_nodes("right", "/dev/video2", (640, 480), 30)
        + generate_camera_nodes("dumper", "/dev/video1")

        # + generate_camera_nodes('front', '/dev/video3')
        # + generate_camera_nodes('left', '/dev/video0')
        # + generate_camera_nodes('right', '/dev/video0')
        # + generate_camera_nodes('dumper', '/dev/video0')
        # + generate_camera_nodes('digger', '/dev/video0')
    )
