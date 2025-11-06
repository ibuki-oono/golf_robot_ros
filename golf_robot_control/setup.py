from setuptools import setup

package_name = 'golf_robot_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ball_follow_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ibuki',
    maintainer_email='ibuki@example.com',
    description='Control robot to follow the ball and kick it',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ball_follow_node = golf_robot_control.ball_follow_node:main',
        ],
    },
)
