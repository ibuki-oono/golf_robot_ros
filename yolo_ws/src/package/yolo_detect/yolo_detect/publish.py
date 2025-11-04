# rclpy (ROS 2のpythonクライアント)の機能を使えるようにします。
import rclpy
# rclpy (ROS 2のpythonクライアント)の機能のうちNodeを簡単に使えるようにします。こう書いていない場合、Nodeではなくrclpy.node.Nodeと書く必要があります。
from rclpy.node import Node

# ROS 2の文字列型を使えるようにimport
from std_msgs.msg import Float32MultiArray

# for yolo package
import cv2
# from ultralytics import YOLO
from yolov8 import YOLOv8
import math

angle = 70
dim = [0,30,180]

# C++と同じく、Node型を継承します。
class YoloPublisher(Node):
    # コンストラクタです、MinimulPublisherクラスのインスタンスを作成する際に呼び出されます。
    def __init__(self):
        # 継承元のクラスを初期化します。（https://www.python-izm.com/advanced/class_extend/）今回の場合継承するクラスはNodeになります。
        super().__init__('yolo_publisher')
        # publisherを作成します。self.と頭についていますが、これはself.publisherがメンバー変数であることを表しています。
        self.publisher_ = self.create_publisher(
            # String型のデータを受信することを示します。
            Float32MultiArray,
            # topicという名前のtopicにデータを送信します。
            'ball',
            # MessageのBuffer Sizeです。基本的には1で問題ありません。
            10)


    # タイマーによって実行されるコールバック
    def publish_message(self, dist, degr):
        # 文字列型のインスタンスを作成します、この変数はローカル変数のため、コールバックの外からはアクセスできません。
        #msg1 = Float32()
        #msg2 = Float32()
        # Stringのメッセージ型の変数に文字列を代入
        #msg1.data = float(dist)
        #msg2.data = float(degr)
        msg = Float32MultiArray()
        msg.data = [dist, degr]
        # Publisher経由でメッセージを発行
        self.publisher_.publish(msg)
        #self.publisher_.publish(msg2)


# mainという名前の関数です。C++のmain関数とは異なり、これは処理の開始地点ではありません。
def main(args=None):
    # rclpyの初期化処理です。ノードを立ち上げる前に実装する必要があります。
    rclpy.init(args=args)
    # MinimalPublisherクラスのインスタンスを作成
    yolo_publisher = YoloPublisher()

    #model = YOLO('/home/ibuntu/runs/detect/train10/weights/best.pt')
    model_path = "/home/ibuntu/runs/detect/train10/weights/best.onnx"
    yolov8_detector = YOLOv8(model_path, conf_thres=0.5, iou_thres=0.5)

    # print(model.names)
    webcamera = cv2.VideoCapture(0)

    try:
        while rclpy.ok():
            success, frame = webcamera.read()
            if success:
                boxes, scores, classes = yolov8_detector(frame)
                #results = yolov8_detector(frame)
                #classes = results[0].boxes.cls
                #boxes = results[0].boxes
                for box, cls in zip(boxes, classes):
                    x1, y1, x2, y2 = [int(i) for i in box]
                    bsize = math.sqrt((x1-x2)*(y1-y2))
                    dist = 23000/bsize

                    degr = math.atan(((x1+x2)/2-260)/((y1+y2)/2-1200))

                    yolo_publisher.publish_message(dist, degr)
                if len(boxes) == 0:
                    yolo_publisher.publish_message(float(0),float(0))
            else:
                yolo_publisher.publish_message(float(0),float(0))
    except KeyboardInterrupt:
        webcamera.release()
    # spin処理を実行、spinをしていないとROS 2のノードはデータを入出力することが出来ません。
    rclpy.spin(yolo_publisher)
    # 明示的にノードの終了処理を行います。
    yolo_publisher.destroy_node()
    # rclpyの終了処理、これがないと適切にノードが破棄されないため様々な不具合が起こります。
    rclpy.shutdown()


# 本スクリプト(publish.py)の処理の開始地点です。
if __name__ == '__main__':
    # 関数`main`を実行する。
    main()
