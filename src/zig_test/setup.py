import os
from glob import glob
from setuptools import setup

package_name = "zig_test"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*launch.[pxy][yma]*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Michael Foley",
    maintainer_email="foley586@umn.edu",
    description="This package is for the zig test subsystem.",
    license="MIT License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "zig_test_node = zig_test.zig_test_node:main",
        ],
    },
)
