from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # NOTE: Instead of an index, you can use the stable path found in /dev/v4l/by-id/ or /dev/v4l/by-path/
    # (e.g.) -> /dev/v4l/by-id/usb-Logitech_Webcam_C920_ABC123-video-index0
    camera_configs = [
        {"name": "right", "topic": "/webcam/right", "id": "/dev/video6"},
        {"name": "left",  "topic": "/webcam/left",  "id": "/dev/video8"},
        {"name": "back",  "topic": "/webcam/back",  "id": "/dev/video0"},
        {"name": "digger", "topic": "/webcam/digger", "id": "/dev/video4"},
        {"name": "dumper",  "topic": "/webcam/dumper",  "id": "/dev/video5"},
        {"name": "front",  "topic": "/webcam/front",  "id": "/dev/video0"},
    ]

    nodes = [
        Node(
            package='camera_interface',
            executable='compression',
            name=f"compressor_{config['name']}",
            parameters=[{
                'topic_name': config['topic'],
                'device_id': config['id']
            }],
            output='screen'
        )
        for config in camera_configs
    ]

    return LaunchDescription(nodes)