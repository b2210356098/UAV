import time
import cv2

cap = cv2.VideoCapture(0)


try:

    success,img = cap.read()

    cv2.imwrite("Cam_test.png",img)
    print("Camera is working correctly")
except Exception as e:
    print(e)
