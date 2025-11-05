#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import numpy as np
import math

class PoleTracker(Node):
    def __init__(self):
        super().__init__('pole_tracker')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(Point, '/pole_pos', 10)

        self.pole_angle_est = math.radians(-90.0)  # initial angle guess (backwards)
        self.omega_deg = 7.0                      # window ±deg around estimate
        self.alpha = 0.3                           # EMA smoothing

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        angles = angle_min + np.arange(len(ranges)) * angle_increment

        # Only keep window around estimated angle
        omega_rad = math.radians(self.omega_deg)
        normalized = np.array([
            self.normalize_angle(a - self.pole_angle_est) for a in angles
        ])
        mask = np.abs(normalized) <= omega_rad

        filtered_ranges = ranges[mask]
        filtered_angles = angles[mask]

        # ✅ Remove invalid LiDAR values: 0.0, inf, nan
        valid_idx = np.where(
            (filtered_ranges > 0.01) &
            (~np.isinf(filtered_ranges)) &
            (~np.isnan(filtered_ranges))
        )[0]

        if len(valid_idx) == 0:
            self.get_logger().warn("No valid pole data in the window.")
            return

        filtered_ranges = filtered_ranges[valid_idx]
        filtered_angles = filtered_angles[valid_idx]

        # nearest point = pole candidate
        min_idx = np.argmin(filtered_ranges)
        pole_distance = float(filtered_ranges[min_idx])
        pole_angle = float(filtered_angles[min_idx])

        # Update estimate with EMA
        self.pole_angle_est = self.normalize_angle(
            (1 - self.alpha) * self.pole_angle_est + self.alpha * pole_angle
        )

        # Publish
        msg_out = Point()
        msg_out.x = pole_distance       # distance [m]
        msg_out.y = self.pole_angle_est # angle [rad]
        msg_out.z = 0.0

        self.publisher.publish(msg_out)

        self.get_logger().info(
            f"Published pole_pos: distance={pole_distance:.2f} m, "
            f"angle={math.degrees(self.pole_angle_est):.2f}°"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PoleTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
