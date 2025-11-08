#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
import serial
import struct
import time

class WheelSerialBridge(Node):
    def __init__(self):
        super().__init__('wheel_serial_bridge')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # Open serial port
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # wait for hardware ready
        self.get_logger().info(f"Serial port opened: {port} @ {baudrate}")

        # Last packet sent
        self.last_packet = None

        # Latest velocities
        self.vel_r = 0
        self.vel_l = 0
        self.vel_kick = 0

        # Subscribe to /cmd_vel
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Subscribe to /kick_vel
        self.sub_kick = self.create_subscription(
            Int16,
            '/kick_vel',
            self.kick_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg: Twist):
        # Map /cmd_vel to hardcoded motor velocities (int16)
        if msg.linear.x > 0.0:
            self.vel_r, self.vel_l = 3200, 3200
        elif msg.linear.x < 0.0:
            self.vel_r, self.vel_l = -3200, -3200
        elif msg.angular.z > 0.0:
            self.vel_r, self.vel_l = 2200, -2200
        elif msg.angular.z < 0.0:
            self.vel_r, self.vel_l = -2200, 2200
        else:
            self.vel_r, self.vel_l = 0, 0

        self.send_packet_if_changed()

    def kick_vel_callback(self, msg: Int16):
        # Update kick velocity
        self.vel_kick = msg.data
        self.send_packet_if_changed()

    def send_packet_if_changed(self):
        # Pack as little-endian 3 x int16
        packet = struct.pack('<hhh', self.vel_r, self.vel_l, self.vel_kick)

        # Only send if packet changed
        if packet != self.last_packet:
            self.ser.write(packet)
            self.ser.flush()
            self.get_logger().info(f"Sent: vel_r={self.vel_r}, vel_l={self.vel_l}, vel_kick={self.vel_kick}")
            self.last_packet = packet

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
