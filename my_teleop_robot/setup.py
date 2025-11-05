from setuptools import setup

package_name = 'my_teleop_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Keyboard teleop for robot',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'teleop_node = my_teleop_robot.teleop_node:main'
        ],
    },
)
