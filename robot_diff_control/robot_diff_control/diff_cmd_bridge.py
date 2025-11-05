import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotDiffControl(Node):
    def __init__(self):
        super().__init__('robot_diff_control')

        # Sub from nav stack / joystick / teleop
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_cb,
            10
        )

        # Pub to ros2_control diff-drive input
        self.cmd_pub = self.create_publisher(
            Twist,
            '/diff_drive_controller/cmd_vel_unstamped',
            10
        )

        self.get_logger().info("robot_diff_control node started.")

    def cmd_cb(self, msg):
        # Forward the message as-is to ros2_control
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotDiffControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
