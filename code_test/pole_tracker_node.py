import rclpy
from rclpy.node import Node
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
            10
        )
        self.publisher = self.create_publisher(Point, '/pole_pos', 10)

        self.pole_angle_est = math.radians(90.0)  # 初期推定角度（ラジアン）
        self.omega_deg = 10.0                     # 検出範囲（±度）
        self.alpha = 0.3                          # 平滑化係数

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        angles = angle_min + np.arange(len(ranges)) * angle_increment

        omega_rad = math.radians(self.omega_deg)
        normalized_angles = np.array([
            self.normalize_angle(a - self.pole_angle_est) for a in angles
        ])
        mask = np.abs(normalized_angles) <= omega_rad

        filtered_ranges = ranges[mask]
        filtered_angles = angles[mask]

        if len(filtered_ranges) == 0:
            self.get_logger().warn('No pole detected in expected range.')
            return

        min_index = np.argmin(filtered_ranges)
        pole_distance = filtered_ranges[min_index]
        pole_angle = filtered_angles[min_index]

        # 推定角度を更新（指数移動平均）
        self.pole_angle_est = self.normalize_angle(
            (1 - self.alpha) * self.pole_angle_est + self.alpha * pole_angle
        )

        # メッセージ作成と送信
        msg_out = Point()
        msg_out.x = float(pole_distance)  # 距離
        msg_out.y = float(self.pole_angle_est)  # 角度（ラジアン）
        msg_out.z = 0.0  # 未使用

        self.publisher.publish(msg_out)
        self.get_logger().info(
            f'Published pole_pos: distance={msg_out.x:.2f} m, angle={math.degrees(msg_out.y):.2f}°'
        )

def main(args=None):
    rclpy.init(args=args)
    node = PoleTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()