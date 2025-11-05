import cv2
from ultralytics import YOLO
import math

angle = 70
dim = [0,30,180]
place_ang = 10

model = YOLO('/home/ibuki/golf_ws/golf_robot_ros/code_test/best.pt')
print(model.names)

# Open two cameras
cam0 = cv2.VideoCapture(0)
cam1 = cv2.VideoCapture(2)

ball0_found = False
ball1_found = False
dist0 = angle0 = 0
dist1 = angle1 = 0

while True:
    ret0, frame0 = cam0.read()
    ret1, frame1 = cam1.read()

    if not ret0 or not ret1:
        print("Camera read error!")
        break

    # Rotate cam1 by 180 degrees
    frame1 = cv2.rotate(frame1, cv2.ROTATE_180)

    # # YOLO on cam0
    # results0 = model(frame0)
    # # YOLO on cam1
    # results1 = model(frame1)

    # --- YOLO on cam0 ---
    results0 = model(frame0)[0]
    boxes0 = results0.boxes

    ball0_found = False
    dist0 = angle0 = 0

    if len(boxes0) > 0:
        # Sort by confidence desc, take highest
        best0 = boxes0[boxes0.conf.argmax()]

        cls0 = int(best0.cls)
        name0 = results0.names[cls0]

        if name0 == "ball":
            ball0_found = True
            x1, y1, x2, y2 = [int(v) for v in best0.xyxy[0]]
            bsize = math.sqrt((x1-x2)*(y1-y2))
            dist0 = 46000/bsize
            angle0 = math.atan(((x1+x2)/2 - 320) / ((y1+y2)/2 - 1400)) / math.pi * 180 + place_ang

    annotated0 = results0.plot()

    # --- YOLO on cam1 ---
    results1 = model(frame1)[0]
    boxes1 = results1.boxes

    ball1_found = False
    dist1 = angle1 = 0

    if len(boxes1) > 0:
        # Sort by confidence desc, take highest
        best1 = boxes1[boxes1.conf.argmax()]

        cls1 = int(best1.cls)
        name1 = results1.names[cls1]

        if name1 == "ball":
            ball1_found = True
            x1, y1, x2, y2 = [int(v) for v in best1.xyxy[0]]
            bsize = math.sqrt((x1-x2)*(y1-y2))
            dist1 = 46000/bsize
            angle1 = math.atan(((x1+x2)/2 - 320) / ((y1+y2)/2 - 1400)) / math.pi * 180 - place_ang

    annotated1 = results1.plot()

    # --- Decision Logic for Display ---
    if ball0_found and ball1_found:
        final_dist = (dist0 + dist1) / 2
        final_angle = (angle0 + angle1) / 2
        text = f"Both cams: dist={final_dist:.1f}mm  ang={final_angle:.1f}deg"

    elif ball0_found and not ball1_found:
        final_dist = dist0
        final_angle = angle0
        text = f"Cam0: dist={final_dist:.1f}mm  ang={final_angle:.1f}deg"

    elif ball1_found and not ball0_found:
        final_dist = dist1
        final_angle = angle1
        text = f"Cam1: dist={final_dist:.1f}mm  ang={final_angle:.1f}deg"

    else:
        text = "NO BALL"

    # --- Combine & Display ---
    combined = cv2.hconcat([annotated0, annotated1])
    cv2.putText(combined, text, (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

    cv2.imshow("YOLO Dual Camera", combined)


    if cv2.waitKey(1) == ord('q'):
        break

cam0.release()
cam1.release()
cv2.destroyAllWindows()
