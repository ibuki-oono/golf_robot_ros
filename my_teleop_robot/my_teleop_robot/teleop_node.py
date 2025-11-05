#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import sys, select, termios, tty

msg = """
Control Your Robot!
---------------------------
Moving around:
   w
a  s  d
   x

Kick control:
k : kick (while pressed)
j : decrease kick value
l : increase kick value

CTRL-C to quit
"""

# Movement keys
move_bindings = {
    'w': (1, 0, 0, 0),
    'x': (-1, 0, 0, 0),
    'a': (0, 0, 0, 1),
    'd': (0, 0, 0, -1),
    's': (0, 0, 0, 0)
}

# Speed adjustment keys
# speed_bindings = {
#     'l': (1.1, 1.1),  # increase
#     'j': (0.9, 0.9)   # decrease
# }

# Kick adjustment keys
kick_adjust = {
    'j': -50,   # decrease kick value
    'l': 50     # increase kick value
}

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopNode(Node):
    def __init__(self):
        super().__init__('my_teleop_robot')
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_kick = self.create_publisher(Int32, '/kick_vel', 10)

        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.kick_value = 900
        self.kick_active = False

    def run(self):
        global settings
        settings = termios.tcgetattr(sys.stdin)
        try:
            print(msg)
            while True:
                key = get_key()

                # Movement
                if key in move_bindings.keys():
                    x, y, z, th = move_bindings[key]
                    twist = Twist()
                    twist.linear.x = x * self.linear_speed
                    twist.linear.y = y * self.linear_speed
                    twist.linear.z = z * self.linear_speed
                    twist.angular.z = th * self.angular_speed
                    self.pub_cmd.publish(twist)

                # Kick button
                self.kick_active = (key == 'k')

                # Kick adjustment
                if key in kick_adjust.keys():
                    self.kick_value += kick_adjust[key]
                    print(f"Kick value: {self.kick_value}")

                # Speed adjustment
                # if key in speed_bindings.keys():
                #     self.linear_speed *= speed_bindings[key][0]
                #     self.angular_speed *= speed_bindings[key][1]
                #     print(f"Speeds updated: linear {self.linear_speed}, angular {self.angular_speed}")

                # CTRL-C
                if key == '\x03':
                    break

                # Publish kick velocity
                kick_msg = Int32()
                kick_msg.data = self.kick_value if self.kick_active else 0
                self.pub_kick.publish(kick_msg)

        except Exception as e:
            print(e)
        finally:
            # Stop robot and kick
            self.pub_cmd.publish(Twist())
            self.pub_kick.publish(Int32(data=0))
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
