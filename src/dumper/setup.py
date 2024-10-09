import os
from glob import glob
from setuptools import setup

package_name = "dumper"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*launch.[pxy][yma]*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Anthony Brogni",
    maintainer_email="brogn002@umn.edu",
    description="This package is for the dumper subsystem.",
    license="MIT License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dumper_node = dumper.dumper_node:main",
        ],
    },
)
