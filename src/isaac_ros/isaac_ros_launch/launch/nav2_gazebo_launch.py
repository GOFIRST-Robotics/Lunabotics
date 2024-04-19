# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory("isaac_ros_launch")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    pkg_robot_description = get_package_share_directory("robot_description")

    # Launch Arguments
    run_rviz_arg = DeclareLaunchArgument("run_rviz", default_value="True", description="Whether to start RVIZ")
    global_frame = LaunchConfiguration("global_frame", default="odom")

    # TODO: Working on adding Nvblox support
    # Create a shared container to hold composable nodes
    # for speed ups through intra process communication.
    # shared_container_name = "shared_nvblox_container"
    # shared_container = Node(
    #     name=shared_container_name,
    #     package="rclcpp_components",
    #     executable="component_container_mt",
    #     output="screen",
    # )

    # Robot State Publisher node (publishing static tfs for the camera)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_robot_description, "launch", "robot_description.launch.py"))
    )

    # TODO: Working on adding Nvblox support
    # nvblox_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(bringup_dir, "nvblox.launch.py")]),
    #     launch_arguments={
    #         "global_frame": global_frame,
    #         "setup_for_gazebo": "True",
    #         "attach_to_shared_component_container": "True",
    #         "component_container_name": shared_container_name,
    #     }.items(),
    # )

    # Rviz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    bringup_dir,
                    "rviz.launch.py",
                )
            ]
        ),
        launch_arguments={
            "config_name": "zed_example.rviz",
            "global_frame": global_frame,
        }.items(),
        condition=IfCondition(LaunchConfiguration("run_rviz")),
    )
    # Nav2
    nav2_param_file = os.path.join("config", "nav2_isaac_sim.yaml")
    param_substitutions = {"global_frame": LaunchConfiguration("global_frame", default="odom")}
    configured_params = RewrittenYaml(
        source_file=nav2_param_file,
        root_key="",
        param_rewrites=param_substitutions,
        convert_types=True,
    )
    # nav2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")),
        launch_arguments={
            "use_sim_time": "False",
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )

    frame_id_renamer = Node(
        package="isaac_ros_launch",
        executable="frame_id_renamer",
        name="frame_id_renamer",
    )

    return LaunchDescription([run_rviz_arg, rviz_launch, nav2_launch, frame_id_renamer, robot_state_publisher])
