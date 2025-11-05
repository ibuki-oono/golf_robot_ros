import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from ultralytics import YOLO
import math


class BallDetectorNode(Node):
    def __init__(self):
        super().__init__('ball_detector')

        self.pub = self.create_publisher(String, '/ball_place', 10)

        self.get_logger().info("Loading YOLO model...")
        self.model = YOLO('/home/ibuki/golf_ws/golf_robot_ros/code_test/best.pt')

        self.cam0 = cv2.VideoCapture(0)
        self.cam1 = cv2.VideoCapture(2)

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.place_ang = 10

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

        if found0 and found1:
            dist = (dist0 + dist1) / 2
            angle = (angle0 + angle1) / 2
            msg = f"ball: distance={dist:.1f}mm angle={angle:.1f}deg"

        elif found0:
            msg = f"ball: distance={dist0:.1f}mm angle={angle0:.1f}deg"

        elif found1:
            msg = f"ball: distance={dist1:.1f}mm angle={angle1:.1f}deg"

        else:
            msg = "ball: NO BALL"

        ros_msg = String()
        ros_msg.data = msg
        self.pub.publish(ros_msg)

        combined = cv2.hconcat([results0.plot(), results1.plot()])
        cv2.putText(combined, msg, (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        cv2.imshow("YOLO Dual Camera", combined)
        cv2.waitKey(1)

    def process_cam(self, results, offset):
        boxes = results.boxes
        if len(boxes) == 0:
            return 0, 0, False
        
        best = boxes[boxes.conf.argmax()]
        if results.names[int(best.cls)] != "ball":
            return 0, 0, False

        x1, y1, x2, y2 = [int(v) for v in best.xyxy[0]]
        bsize = math.sqrt((x1-x2)*(y1-y2))
        dist = 46000 / bsize
        angle = math.atan(((x1+x2)/2 - 320) / ((y1+y2)/2 - 1400)) / math.pi * 180 + offset

        return dist, angle, True


def main(args=None):
    rclpy.init(args=args)
    node = BallDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
