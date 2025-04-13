from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

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

    calibrate_field_coordinate_server = Node(
        package="rovr_control",
        executable="calibrate_field_coordinate_server",
        name="calibrate_field_coordinate_server",
    )

    dig_location_server = Node(
        package="rovr_control",
        executable="dig_location_server",
        name="dig_location_server",
    )
    
    behaviortree_executor = Node(
        package="behaviortree_executor",
        executable="behaviortree_executor",
        name="bt_action_server",
        output="screen",
        parameters=[
            PathJoinSubstitution([
            FindPackageShare('behaviortree_executor'),
            'config',
            'sample_bt_executor.yaml'
        ]), 
    ],
        emulate_tty=True,
    )
    ld.add_action(auto_dig_server)
    ld.add_action(auto_offload_server)
    ld.add_action(calibrate_field_coordinate_server)
    ld.add_action(dig_location_server)
    ld.add_action(behaviortree_executor)
    

    return ld
