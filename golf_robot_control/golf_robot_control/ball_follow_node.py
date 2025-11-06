import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Int16
import time

class BallFollowNode(Node):
    def __init__(self):
        super().__init__('ball_follow_node')

        # Subscribers
        self.sub = self.create_subscription(Point, '/ball_place', self.ball_callback, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.kick_pub = self.create_publisher(Int16, '/kick_vel', 10)

        self.angle_tolerance = 0.1
        self.straight_speed = 0.3
        self.turn_speed = 1.0
        self.kick_power = 1200

        self.last_ball = Point()
        self.kick_flag = False
        self.kick_start_time = None
        self.straight_start_time = None

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

    def ball_callback(self, msg):
        self.last_ball = msg

    def timer_callback(self):
        twist = Twist()

        angle = self.last_ball.y
        distance = self.last_ball.x

        # No ball detected (distance = 0)
        if distance == 0.0:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return

        # Turn to align with ball
        if angle > self.angle_tolerance:
            twist.angular.z = self.turn_speed
            twist.linear.x = 0.0
        elif angle < -self.angle_tolerance:
            twist.angular.z = -self.turn_speed
            twist.linear.x = 0.0
        else:
            twist.angular.z = 0.0
            # Move straight if aligned
            if distance > 0.4:
                twist.linear.x = self.straight_speed
            else:
                # Move straight for 2 seconds and then kick
                if self.straight_start_time is None:
                    self.straight_start_time = self.get_clock().now()
                    twist.linear.x = self.straight_speed
                else:
                    elapsed = (self.get_clock().now() - self.straight_start_time).nanoseconds / 1e9
                    if elapsed < 2.0:
                        twist.linear.x = self.straight_speed
                    elif not self.kick_flag:
                        # Kick for 1 second
                        self.kick_pub.publish(Int16(data=self.kick_power))
                        self.kick_flag = True
                        self.kick_start_time = self.get_clock().now()
                        twist.linear.x = 0.0
                    elif self.kick_flag:
                        kick_elapsed = (self.get_clock().now() - self.kick_start_time).nanoseconds / 1e9
                        if kick_elapsed >= 1.0:
                            self.kick_flag = False
                            self.straight_start_time = None
                        twist.linear.x = 0.0

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = BallFollowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
