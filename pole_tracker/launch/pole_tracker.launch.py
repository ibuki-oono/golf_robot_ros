from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pole_tracker',
            executable='pole_tracker',
            output='screen',
        )
    ])
