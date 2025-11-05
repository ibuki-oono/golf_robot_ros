#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time

class WheelSerialBridge(Node):
    def __init__(self):
        super().__init__('wheel_serial_bridge')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('left_wheel_joint', 'left_wheel_joint')
        self.declare_parameter('right_wheel_joint', 'right_wheel_joint')
        self.declare_parameter('send_interval', 0.5)  # seconds

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.left_joint = self.get_parameter('left_wheel_joint').get_parameter_value().string_value
        self.right_joint = self.get_parameter('right_wheel_joint').get_parameter_value().string_value
        self.send_interval = self.get_parameter('send_interval').get_parameter_value().double_value

        # Open serial port
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # wait 0.5s for hardware ready
        self.get_logger().info(f"Serial port opened: {port} @ {baudrate}")

        # Store latest wheel velocities
        self.vel_l = 0.0
        self.vel_r = 0.0

        # Subscribe to joint_states
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer to send serial data at fixed interval
        self.timer = self.create_timer(self.send_interval, self.send_serial_command)

    def joint_state_callback(self, msg: JointState):
        # Update latest velocities
        for i, name in enumerate(msg.name):
            if name == self.left_joint:
                self.vel_l = int(msg.velocity[i] * 50)
            elif name == self.right_joint:
                self.vel_r = int(msg.velocity[i] * 50)

    def send_serial_command(self):
        # Send the latest velocity once
        cmd = f"VEL,{self.vel_r},{self.vel_l},0\n"
        # cmd = f"VEL,1666,1666,0\n"
        self.ser.write(cmd.encode('utf-8'))
        # self.ser.flush()
        self.get_logger().debug(f"Sent: {cmd.strip()}")

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
