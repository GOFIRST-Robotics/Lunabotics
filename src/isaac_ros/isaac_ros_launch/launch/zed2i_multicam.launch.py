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


from datetime import datetime

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import ComposableNodeContainer
from launch.conditions import IfCondition
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnShutdown

from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Locate our config files
    config_dir = PathJoinSubstitution(["config", "sensors"])
    config_file_camera = PathJoinSubstitution([config_dir, "zed2i.yaml"])
    config_file_common = PathJoinSubstitution([config_dir, "zed_common.yaml"])

    record_svo_arg = DeclareLaunchArgument(
        "record_svo",
        default_value="False",
        description="Whether to record ZED data to an SVO file",
    )

    # Robot State Publisher node (publishing static tfs for the camera)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("robot_description"), "launch", "robot_description.launch.py"])
        ),
        launch_arguments={
            "setup_for_gazebo": "False",
        }.items(),
    )

    # ZED Wrapper component front
    zed_wrapper_component_front = ComposableNode(
        package="zed_components",
        namespace="zed2i",
        plugin="stereolabs::ZedCamera",
        name="zed_node_front",
        parameters=[
            # YAML files
            config_file_common,  # Common parameters
            config_file_camera,  # Camera related parameters
            {
                'serial_number': 0
            },
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # ZED Wrapper component rear
    zed_wrapper_component_rear = ComposableNode(
        package="zed_components",
        namespace="zed2i",
        plugin="stereolabs::ZedCamera",
        name="zed_node_rear",
        parameters=[
            # YAML files
            config_file_common,  # Common parameters
            config_file_camera,  # Camera related parameters
            {
                'serial_number': 0
            },
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )


    # ROS 2 Component Container
    zed_container = ComposableNodeContainer(
        name="zed_container",
        namespace="zed2i",
        package="rclcpp_components",
        executable="component_container_isolated",
        composable_node_descriptions=[zed_wrapper_component_front, zed_wrapper_component_rear],
        output="screen",
    )

    # Get the current date and time
    now = datetime.now()
    # Format as a string
    timestamp_str = now.strftime("%m-%d-%Y_%H:%M:%S")
    # Add the timestamp to the svo filename
    svo_filename = f"/rosbags/zed_recording_{timestamp_str}.svo2"

    # FRONT
    record_svo_srv_front = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " service call ",
                "/zed2i/zed_node_front/start_svo_rec ",
                "zed_msgs/srv/StartSvoRec ",
                # Tune this bitrate to adjust file size
                f"\"{{compression_mode: 2, bitrate: 10000, svo_filename: '{svo_filename}'}}\"",
            ]
        ],
        shell=True,
        condition=IfCondition(LaunchConfiguration("record_svo")),
    )

    stop_svo_recording_front = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=ExecuteProcess(
                cmd=[
                    [
                        FindExecutable(name="ros2"),
                        " service call ",
                        "/zed2i/zed_node_front/stop_svo_rec ",
                        "std_srvs/srv/Trigger ",
                    ]
                ],
                shell=True,
                condition=IfCondition(LaunchConfiguration("record_svo")),
            )
        )
    )

    # REAR
    record_svo_srv_rear = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " service call ",
                "/zed2i/zed_node_rear/start_svo_rec ",
                "zed_msgs/srv/StartSvoRec ",
                # Tune this bitrate to adjust file size
                f"\"{{compression_mode: 2, bitrate: 10000, svo_filename: '{svo_filename}'}}\"",
            ]
        ],
        shell=True,
        condition=IfCondition(LaunchConfiguration("record_svo")),
    )

    stop_svo_recording_rear = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=ExecuteProcess(
                cmd=[
                    [
                        FindExecutable(name="ros2"),
                        " service call ",
                        "/zed2i/zed_node_rear/stop_svo_rec ",
                        "std_srvs/srv/Trigger ",
                    ]
                ],
                shell=True,
                condition=IfCondition(LaunchConfiguration("record_svo")),
            )
        )
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1"),
            record_svo_arg,
            record_svo_srv_front,
            stop_svo_recording_front,
            record_svo_srv_rear,
            stop_svo_recording_rear,       
            robot_state_publisher,
            zed_container,
        ]
    )
