import os
from glob import glob
from setuptools import setup, find_packages

package_name = "rovr_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=['test']),    
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name),
            glob("launch/*launch.[pxy][yma]*", recursive=True),
        ),
        (os.path.join('share', package_name, 'action_servers'), glob(os.path.join('rovr_control/action_servers', '*')))
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Anthony",
    maintainer_email="anthonybrog@gmail.com",
    description="This package is for the main robot control loop.",
    license="MIT License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            {
                "main_control_node = rovr_control.main_control_node:main",
                "read_serial = rovr_control.read_serial:main",
                
                "calibrate_field_coordinate_server = rovr_control.action_servers.calibrate_field_coordinate_server:main",
                "auto_dig_server = rovr_control.action_servers.auto_dig_server:main",
                "auto_offload_server = rovr_control.action_servers.auto_offload_server:main",
                "dig_location_server = rovr_control.action_servers.dig_location_server:main",
                "auto_dig_nav_offload_server = rovr_control.action_servers.auto_dig_nav_offload_server:main",
                
                "stream_deck_node = rovr_control.stream_deck_node:main",
                "action_server_launch = rovr_control.action_server_launch:main"
            }
        ],
    },
)
