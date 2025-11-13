from setuptools import setup

package_name = 'rqt_camera_old'

setup(
    name=package_name,
    version='1.7.1',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/resource',
            ['resource/cameras.ui']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Adam Yaj',
    license='BSD',
    entry_points={
        'console_scripts': [
            'rqt_camera = ' + package_name + '.main:main',
        ],
    },
)
