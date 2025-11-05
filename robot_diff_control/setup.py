from setuptools import setup
from glob import glob
import os

package_name = 'robot_diff_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Install package.xml
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch & config folders (optional but recommended)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Bridge cmd_vel to ros2_control diffdrive input',
    license='MIT',
    entry_points={
        'console_scripts': [
            'robot_diff_control = robot_diff_control.diff_cmd_bridge:main',
        ],
    },
)
