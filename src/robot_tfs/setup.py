from setuptools import setup

package_name = 'robot_tfs'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jonathan Blixt',
    maintainer_email='blixt013@umn.edu',
    description='TF Package for SLAM',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam_tf = robot_tfs.cam_tf:main'
        ],
    },
)
