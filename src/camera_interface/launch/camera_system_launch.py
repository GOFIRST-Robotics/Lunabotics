from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # NOTE: Instead of an index, you can use the stable path found in /dev/v4l/by-id/ or /dev/v4l/by-path/
    # To get the list of ids, we do : ls -l /dev/v4l/by-id/
    # (e.g.) -> /dev/v4l/by-id/usb-Logitech_Webcam_C920_ABC123-video-index0
    camera_configs = [
        {"name": "right", "topic": "/webcam/right", "id": "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB2.0_FHD_UVC_WebCam-video-index0"},
        {"name": "left",  "topic": "/webcam/left",  "id": "/dev/v4l/by-id/usb-OmniVision_Technologies__Inc._2000-video-index0"}, #
        {"name": "back",  "topic": "/webcam/back",  "id": "/dev/v4l/by-id/usb-OmniVision_Technologies__Inc._USB_Camera-B4.09.24.1-video-index0"},
        {"name": "digger", "topic": "/webcam/digger", "id": "/dev/v4l/by-id/usb-Technologies__Inc._ZED_2i_OV0001-video-index0"},
        {"name": "digger", "topic": "/webcam/digger_2", "id": "/dev/v4l/by-id/usb-Technologies__Inc._ZED_2i_OV0001-video-index1"},
        # {"name": "dumper",  "topic": "/webcam/dumper",  "id": "/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index1"},
        # {"name": "front",  "topic": "/webcam/front",  "id": "/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9782_USB_Camera_UC852-video-index1"},
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