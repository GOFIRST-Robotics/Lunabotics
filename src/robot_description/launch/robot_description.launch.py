from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_project_description = get_package_share_directory("robot_description")
        # Load the SDF file from "description" package
    urdf_file = os.path.join(pkg_project_description, "models", "master_ASM", "urdf", "master_ASM.urdf")
    with open(urdf_file, "r") as infp:
        robot_desc = infp.read()
    
    robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="robot_state_publisher",
    output="both",
    parameters=[
        {"use_sim_time": True},
        {"robot_description": robot_desc},
    ],
    )
    
    return LaunchDescription([robot_state_publisher])