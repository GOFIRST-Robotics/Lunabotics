from setuptools import find_packages, setup
import os
from glob import glob

package_name = "alexblox"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name),
            glob("launch/*launch.[pxy][yma]*", recursive=True),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alex Berg",
    maintainer_email="aaberg333@gmail.com",
    description="TODO: Package description",
    license="MIT License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "costmapGenerator = alexblox.costmapGenerator:main",
            "visualizer = alexblox.visualizer:main",
        ],
    },
)
