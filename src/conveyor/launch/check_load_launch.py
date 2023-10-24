from launch import LaunchDescription
from launch.substitution import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    realsense = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="realsense2_camera",
        remappings=[
            ('/camera/camera/depth/image_rect_raw', '/depth/image_rect_raw'),
            ('/camera/depth/image_rect_raw', '/depth/image_rect_raw'),
            ('/depth/image_rect_raw', '/depth/image_rect_raw'),
        ],
        parameters=[{'depth_module.profile': LaunchConfiguration('848,640,30')}]
    )
    
    check_load = Node(
        package="conveyor",
        executable="ros_check_load",
        name="ros_check_load",
    )
    
    ld.add_action(realsense)
    ld.add_action(check_load)

    return ld