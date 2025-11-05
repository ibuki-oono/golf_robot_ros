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

        # Subscribe to LiDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        # Publisher: generalized pole data
        self.publisher = self.create_publisher(Point, '/pole_pos', 10)

        # Internal state
        self.pole_angle_est = math.radians(-90.0)  # initial guess
        self.omega_deg = 10.0                      # detection window ±deg
        self.alpha = 0.95                          # EMA for angle only

        # Angle center for normalization
        self.angle_center_deg = -90.0              # angle that maps to 0
        self.angle_center_rad = math.radians(self.angle_center_deg)

    def normalize_angle(self, angle):
        """Wrap angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def scan_callback(self, msg: LaserScan):
        # --- get scan angles and ranges ---
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # --- select points within detection window ---
        omega_rad = math.radians(self.omega_deg)
        normalized = np.array([
            self.normalize_angle(a - self.pole_angle_est) for a in angles
        ])
        mask = np.abs(normalized) <= omega_rad

        filtered_ranges = ranges[mask]
        filtered_angles = angles[mask]

        # --- remove invalid points ---
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

        # --- find nearest point ---
        min_idx = np.argmin(filtered_ranges)
        pole_distance = float(filtered_ranges[min_idx])
        detected_angle = float(filtered_angles[min_idx])

        # --- smooth angle only ---
        self.pole_angle_est = self.normalize_angle(
            (1 - self.alpha) * self.pole_angle_est + self.alpha * detected_angle
        )

        # --- generalize angle relative to center ---
        relative_angle = self.normalize_angle(self.pole_angle_est - self.angle_center_rad)

        # --- publish generalized data ---
        msg_out = Point()
        msg_out.x = pole_distance      # raw distance in meters
        msg_out.y = relative_angle     # angle centered at -90°
        msg_out.z = 0.0

        self.publisher.publish(msg_out)

        self.get_logger().info(
            f"Published pole_pos: distance={pole_distance:.2f} m, "
            f"angle={math.degrees(relative_angle):.2f}° (centered at {self.angle_center_deg}°)"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PoleTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
