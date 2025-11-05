#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct
import time

class WheelSerialBridge(Node):
    def __init__(self):
        super().__init__('wheel_serial_bridge')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # Open serial port
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # wait for hardware ready
        self.get_logger().info(f"Serial port opened: {port} @ {baudrate}")

        # Last command sent
        self.last_packet = None

        # Subscribe to cmd_vel
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg: Twist):
        # Map /cmd_vel to hardcoded motor velocities (int16)
        if msg.linear.x > 0.0:
            vel_r, vel_l, vel_kick = 1800, 1800, 0
        elif msg.linear.x < 0.0:
            vel_r, vel_l, vel_kick = -1800, -1800, 0
        elif msg.angular.z > 0.0:
            vel_r, vel_l, vel_kick = 1800, -1800, 0
        elif msg.angular.z < 0.0:
            vel_r, vel_l, vel_kick = -1800, 1800, 0
        else:
            vel_r, vel_l, vel_kick = 0, 0, 0

        # Pack as little-endian 3 x int16
        packet = struct.pack('<hhh', vel_r, vel_l, vel_kick)

        # Only send if different from last packet
        # if packet != self.last_packet:
        self.ser.write(packet)
        self.ser.flush()
        self.get_logger().info(f"Sent: {vel_r}, {vel_l}, {vel_kick}")
            # self.last_packet = packet

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
