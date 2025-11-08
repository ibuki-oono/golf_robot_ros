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

        # Parameters
        self.angle_tolerance = 0.06
        self.straight_speed = 0.3
        self.turn_speed = 1.0
        self.kick_power = 1500

        # Ball data
        self.last_ball = Point()
        self.filtered_distance = 0.0
        self.rc_alpha = 0.7  # RC filter coefficient
        self.prev_ball = Point()
        self.last_change_time = self.get_clock().now()
        self.timeout_sec = 3.0

        # Kick logic
        self.kick_flag = False
        self.kick_start_time = None
        self.straight_start_time = None
        self.last_kick_time = self.get_clock().now()

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

    def ball_callback(self, msg):
        # Check if ball data changed significantly
        angle_changed = abs(msg.y - self.prev_ball.y) > 1e-3
        distance_changed = abs(msg.x - self.prev_ball.x) > 1e-3

        if angle_changed or distance_changed:
            self.last_change_time = self.get_clock().now()
            self.prev_ball = msg

        # Apply RC filter for distance
        self.filtered_distance = self.rc_alpha * msg.x + (1 - self.rc_alpha) * self.filtered_distance

        # Store filtered data
        self.last_ball.y = msg.y
        self.last_ball.x = self.filtered_distance

    def timer_callback(self):
        twist = Twist()

        # Timeout: if data unchanged > 3 sec, modify distance/angle
        elapsed = (self.get_clock().now() - self.last_change_time).nanoseconds / 1e9
        if elapsed > self.timeout_sec:
            distance = 1.0
            if self.last_ball.y >= 0:
                angle = 0.3
            else:
                angle = -0.3
        else:
            distance = self.last_ball.x
            angle = self.last_ball.y

        # Turn to align with ball
        if angle > self.angle_tolerance:
            twist.angular.z = self.turn_speed
            twist.linear.x = 0.0
        elif angle < -self.angle_tolerance:
            twist.angular.z = -self.turn_speed
            twist.linear.x = 0.0
        else:
            twist.angular.z = 0.0

            # If distance < 0.5, stop and then go straight for 3 seconds
            if distance < 0.4:
                twist.linear.x = self.straight_speed  # Approach slowly
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                time.sleep(6)  # Small delay to ensure stop
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                time.sleep(0.5)
                self.kick_pub.publish(Int16(data=self.kick_power))
                time.sleep(1)
                self.kick_pub.publish(Int16(data=0))
            #    self.last_change_time = self.get_clock().now()
            #     if self.straight_start_time is None:
            #         # Start moving straight for 3 seconds
            #         self.straight_start_time = self.get_clock().now()
            #         twist.linear.x = self.straight_speed
            #         twist.angular.z = 0.0
            #     else:
            #         elapsed_straight = (self.get_clock().now() - self.straight_start_time).nanoseconds / 1e9
            #         if elapsed_straight < 5.0:  # Move straight for 3 seconds
            #             twist.linear.x = self.straight_speed
            #             twist.angular.z = 0.0
            #         elif elapsed_straight >= 5.0 and (self.get_clock().now() - self.last_kick_time).nanoseconds / 1e9 >= 5.0:
            #             # Send kick for 1 second
            #             self.kick_pub.publish(Int16(data=self.kick_power))
            #             self.kick_flag = True
            #             self.kick_start_time = self.get_clock().now()
            #             twist.linear.x = 0.0
            #             twist.angular.z = 0.0
            #             self.last_kick_time = self.get_clock().now()
            #         elif self.kick_flag:
            #             kick_elapsed = (self.get_clock().now() - self.kick_start_time).nanoseconds / 1e9
            #             if kick_elapsed >= 0.5:
            #                 self.kick_flag = False
            #                 self.straight_start_time = None
            #                 self.kick_pub.publish(Int16(data=0))
            #             twist.linear.x = 0.0
            #             twist.angular.z = 0.0
            else:
                # Move straight if aligned and distance > 0.5
                twist.linear.x = self.straight_speed
                self.straight_start_time = None

        self.cmd_pub.publish(twist)
        self.kick_pub.publish(Int16(data=0))


def main(args=None):
    rclpy.init(args=args)
    node = BallFollowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
