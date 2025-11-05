from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='ball_detector',
            executable='ball_node',
            name='ball_detector',
            output='screen'
        )
    ])
