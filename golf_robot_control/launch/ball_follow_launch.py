from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='golf_robot_control',
            executable='ball_follow_node',
            name='ball_follow_node',
            output='screen'
        )
    ])
