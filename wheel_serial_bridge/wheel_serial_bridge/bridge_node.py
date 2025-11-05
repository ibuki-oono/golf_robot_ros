#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class WheelSerialBridge(Node):
    def __init__(self):
        super().__init__('wheel_serial_bridge')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('send_interval', 0.1)  # check every 0.1 s

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.send_interval = self.get_parameter('send_interval').get_parameter_value().double_value

        # Open serial port
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # wait for hardware ready
        self.get_logger().info(f"Serial port opened: {port} @ {baudrate}")

        # Last command sent
        self.last_cmd = None

        # Subscribe to cmd_vel
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg: Twist):
        # Map /cmd_vel to hardcoded motor commands
        if msg.linear.x > 0.0:
            cmd = "VEL,2000,2000,0\r\n"
        elif msg.linear.x < 0.0:
            cmd = "VEL,-2000,-2000,0\r\n"
        elif msg.angular.z > 0.0:
            cmd = "VEL,-2000,2000,0\r\n"
        elif msg.angular.z < 0.0:
            cmd = "VEL,2000,-2000,0\r\n"
        else:
            cmd = "VEL,0,0,0\r\n"

        # Only send if it is different from last command
        if cmd != self.last_cmd:
            self.ser.write(cmd.encode('utf-8'))
            self.ser.flush()  # ensure hardware receives it immediately
            self.get_logger().info(f"Sent: {cmd.strip()}")
            self.last_cmd = cmd

def main(args=None):
    rclpy.init(args=args)
    node = WheelSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.ser.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
