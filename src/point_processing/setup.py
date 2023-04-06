from setuptools import setup

package_name = 'point_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eric Patton',
    maintainer_email='3rix40@gmail.com',
    description='Processing updating point maps for use in path finding',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointcloud_subscriber = point_processing.gen_grid_world:main',
            'pointcloud_test_publisher = point_processing.test_publisher:main',
        ],
    },
)
