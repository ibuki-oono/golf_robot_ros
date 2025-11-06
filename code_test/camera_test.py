import cv2

def list_camera_indices(max_indices=10):
    available = []
    for i in range(max_indices):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            available.append(i)
            cap.release()
    return available

print("Available camera indices:", list_camera_indices())
