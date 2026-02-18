from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    # Nav2 params
    nav2_param_file = PathJoinSubstitution(["config", "nav2_isaac_sim.yaml"])
    param_substitutions = {
        "global_frame": LaunchConfiguration("global_frame", default="odom")
    }
    configured_params = RewrittenYaml(
        source_file=nav2_param_file,
        root_key="",
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    # nav2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )

    return LaunchDescription([nav2_launch])
