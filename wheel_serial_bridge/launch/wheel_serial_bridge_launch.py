from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wheel_serial_bridge',
            executable='bridge_node',
            name='wheel_serial_bridge',
            output='screen',
            parameters=[
                {'port': '/dev/ttyACM1'},
                {'baudrate': 115200},
                {'left_wheel_joint': 'left_wheel_joint'},
                {'right_wheel_joint': 'right_wheel_joint'}
            ]
        )
    ])
