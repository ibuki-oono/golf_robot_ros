import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import cv2
from ultralytics import YOLO
import math


class BallDetectorNode(Node):
    def __init__(self):
        super().__init__('ball_detector')

        # Publisher using Point message
        self.pub = self.create_publisher(Point, '/ball_place', 10)

        self.get_logger().info("Loading YOLO model...")
        self.model = YOLO('/home/ibuki/golf_ws/golf_robot_ros/code_test/best.pt')

        # Cameras
        self.cam0 = cv2.VideoCapture(0)
        self.cam1 = cv2.VideoCapture(2)

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.place_ang = 10

        # Range filter state
        self.prev_distance = 0.0
        self.prev_angle = 0.0
        self.angle_tolerance = 10.0  # degrees

    def timer_callback(self):
        ret0, frame0 = self.cam0.read()
        ret1, frame1 = self.cam1.read()

        if not ret0 or not ret1:
            return

        frame1 = cv2.rotate(frame1, cv2.ROTATE_180)

        results0 = self.model(frame0)[0]
        results1 = self.model(frame1)[0]

        dist0, angle0, found0 = self.process_cam(results0, +self.place_ang)
        dist1, angle1, found1 = self.process_cam(results1, -self.place_ang)

        # Decide which camera result to use
        if found0 and found1:
            distance = (dist0 + dist1) / 2 / 1000  # meters
            angle = (angle0 + angle1) / 2
        elif found0:
            distance = dist0 / 1000
            angle = angle0
        elif found1:
            distance = dist1 / 1000
            angle = angle1
        else:
            # No ball found
            distance = self.prev_distance
            angle = self.prev_angle

        # --- range filter ---
        if abs(angle - self.prev_angle) <= self.angle_tolerance:
            # Accept new data
            self.prev_distance = distance
            self.prev_angle = angle
        else:
            # Reject new data, keep previous
            distance = self.prev_distance
            angle = self.prev_angle

        # --- publish generalized data ---
        msg_out = Point()
        msg_out.x = distance                       # distance in meters
        msg_out.y = angle / 180 * math.pi          # angle in radians
        msg_out.z = 0.0

        self.pub.publish(msg_out)

    def process_cam(self, results, offset):
        boxes = results.boxes
        if len(boxes) == 0:
            return 0, 0, False

        best = boxes[boxes.conf.argmax()]
        if results.names[int(best.cls)] != "ball":
            return 0, 0, False

        x1, y1, x2, y2 = [int(v) for v in best.xyxy[0]]
        bsize = math.sqrt((x1 - x2) * (y1 - y2))
        dist = 46000 / bsize                # mm
        angle = math.atan(((x1 + x2) / 2 - 320) / ((y1 + y2) / 2 - 1400)) / math.pi * 180 + offset

        return dist, angle, True


def main(args=None):
    rclpy.init(args=args)
    node = BallDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
