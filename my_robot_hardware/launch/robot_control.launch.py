from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                'urdf/my_robot_hardware.urdf',       # your robot description
                'config/my_robot_hardware_params.yaml',  # hardware params including serial
                'config/diff_drive_controller.yaml'   # controller config
            ],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        )
    ])
