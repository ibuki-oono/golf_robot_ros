from setuptools import setup
import os

package_name = 'wheel_serial_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='ibuki',
    maintainer_email='ibuki@example.com',
    description='ROS2 node to send wheel velocities over serial from joint_states',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'bridge_node = wheel_serial_bridge.bridge_node:main',
        ],
    },
    data_files=[
        # Install package.xml in the share directory
        (os.path.join('share', package_name), ['package.xml']),
        # Optionally install launch files
        (os.path.join('share', package_name, 'launch'), ['launch/wheel_serial_bridge_launch.py']),
    ],
)
