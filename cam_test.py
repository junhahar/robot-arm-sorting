import cv2
for i in range(5):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        ret, frame = cap.read()
        print(f"CAM {i}: opened={'Y'}, read={'Y' if ret else 'N'}, size={frame.shape if ret else 'N/A'}")
    else:
        print(f"CAM {i}: opened=N")
    cap.release()
