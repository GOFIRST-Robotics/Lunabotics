# Copyright 2024 Stereolabs
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from datetime import datetime

from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import ComposableNodeContainer

from launch.substitutions import FindExecutable
from launch.conditions import IfCondition
from launch.actions import (
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown


def parse_array_param(param):
    str = param.replace("[", "")
    str = str.replace("]", "")
    str = str.replace(" ", "")
    arr = str.split(",")

    return arr


def launch_setup(context, *args, **kwargs):
    # Locate our config files
    config_dir = PathJoinSubstitution(["config", "sensors"])
    config_file_common = PathJoinSubstitution([config_dir, "zed_common.yaml"])

    # List of actions to be launched
    actions = []

    namespace_val = "zed_multi"

    names = LaunchConfiguration("cam_names")
    models = LaunchConfiguration("cam_models")
    serials = LaunchConfiguration("cam_serials")
    disable_tf = LaunchConfiguration("disable_tf")

    names_arr = parse_array_param(names.perform(context))
    models_arr = parse_array_param(models.perform(context))
    serials_arr = parse_array_param(serials.perform(context))
    disable_tf_val = disable_tf.perform(context)

    num_cams = len(names_arr)

    if num_cams != len(models_arr):
        return [
            LogInfo(
                msg=TextSubstitution(
                    text="The size of the `models` param array must be equal to the size of `names`"
                )
            )
        ]

    if num_cams != len(serials_arr):
        return [
            LogInfo(
                msg=TextSubstitution(
                    text="The size of the `serials` param array must be equal to the size of `names`"
                )
            )
        ]

    # ROS 2 Component Container
    container_name = "zed_multi_container"
    distro = os.environ["ROS_DISTRO"]
    if distro == "foxy":
        # Foxy does not support the isolated mode
        container_exec = "component_container"
    else:
        container_exec = "component_container_isolated"

    info = (
        "* Starting Composable node container: /" + namespace_val + "/" + container_name
    )
    actions.append(LogInfo(msg=TextSubstitution(text=info)))

    zed_container = ComposableNodeContainer(
        name=container_name,
        namespace=namespace_val,
        package="rclcpp_components",
        executable=container_exec,
        arguments=["--ros-args", "--log-level", "info"],
        output="screen",
    )
    actions.append(zed_container)

    # Get the current date and time
    now = datetime.now()
    # Format as a string
    timestamp_str = now.strftime("%m-%d-%Y_%H:%M:%S")

    # Set the first camera idx
    cam_idx = 0

    for name in names_arr:
        model = models_arr[cam_idx]
        serial = serials_arr[cam_idx]

        info = (
            "* Starting a ZED ROS2 node for camera "
            + name
            + " ("
            + model
            + "/"
            + serial
            + ")"
        )

        actions.append(LogInfo(msg=TextSubstitution(text=info)))

        # Only the first camera send odom and map TF
        publish_tf = "false"
        if cam_idx == 0:
            if disable_tf_val == "False" or disable_tf_val == "false":
                publish_tf = "true"

        # A different node name is required by the Diagnostic Updated
        node_name = "zed_node_" + name

        # Add the node
        # ZED Wrapper launch file
        zed_wrapper_launch = IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                [
                    get_package_share_directory("zed_wrapper"),
                    "/launch/zed_camera.launch.py",
                ]
            ),
            launch_arguments={
                "container_name": container_name,
                "node_name": node_name,
                "camera_name": name,
                "camera_model": model,
                "serial_number": serial,
                "publish_tf": publish_tf,
                "publish_map_tf": publish_tf,
                "namespace": namespace_val,
                "config_path": config_file_common,
            }.items(),
        )
        actions.append(zed_wrapper_launch)

        # Only record an SVO from the first camera
        if cam_idx == 0:
            # Add the timestamp to the svo filename
            svo_filename = f"/rosbags/{name}_recording_{timestamp_str}.svo2"
            # Record an SVO2 file
            record_svo_srv = ExecuteProcess(
                cmd=[
                    [
                        FindExecutable(name="ros2"),
                        " service call ",
                        f"/{name}/{node_name}/start_svo_rec ",
                        "zed_msgs/srv/StartSvoRec ",
                        # Tune this bitrate to adjust file size
                        f"\"{{compression_mode: 2, bitrate: 10000, svo_filename: '{svo_filename}'}}\"",
                    ]
                ],
                shell=True,
                condition=IfCondition(LaunchConfiguration("record_svo")),
            )
            actions.append(record_svo_srv)
            # Stop recording the SVO2 file
            stop_svo_recording = RegisterEventHandler(
                event_handler=OnShutdown(
                    on_shutdown=ExecuteProcess(
                        cmd=[
                            [
                                FindExecutable(name="ros2"),
                                " service call ",
                                f"/{name}/{node_name}/stop_svo_rec ",
                                "std_srvs/srv/Trigger ",
                            ]
                        ],
                        shell=True,
                        condition=IfCondition(LaunchConfiguration("record_svo")),
                    )
                )
            )
            actions.append(stop_svo_recording)

        cam_idx += 1

    # Robot State Publisher node (publishing static tfs for the camera)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_description"),
                    "launch",
                    "robot_description.launch.py",
                ]
            )
        ),
        launch_arguments={
            "setup_for_gazebo": "False",
        }.items(),
    )

    # Add the robot_state_publisher node to the list of nodes to be started
    actions.append(robot_state_publisher)

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "record_svo",
                default_value="False",
                description="Whether to record ZED data to an SVO file",
            ),
            DeclareLaunchArgument(
                "cam_names",
                default_value="[zed2i,zed2i_rear]",
                description="An array containing the names of the cameras, e.g. [zed_front,zed_back]",
            ),
            DeclareLaunchArgument(
                "cam_models",
                default_value="[zed2i,zed2i]",
                description="An array containing the names of the cameras, e.g. [zed2i,zed2]",
            ),
            DeclareLaunchArgument(
                "cam_serials",
                default_value="[38536461,32113890]",
                description="An array containing the serial numbers of the cameras, e.g. [35199186,23154724]",
            ),
            DeclareLaunchArgument(
                "disable_tf",
                default_value="False",
                description="If `True` disable TF broadcasting for all of the cameras.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
