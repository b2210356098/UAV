import cv2
import numpy as np
import matplotlib.pyplot as plt

"""
camera = cv2.VideoCapture(0)
return_value, image = camera.read()
cv2.imwrite("deneme.png", image)

"""


def sum_filter(name)  :
    img = cv2.imread(name)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 50, 70])
    upper_red = np.array([9, 255,255])

    mask = cv2.inRange(hsv, lower_red, upper_red)
    res = cv2.bitwise_and(img, img, mask=mask)

    cv2.imwrite("deneme2.png",res)

    img_red = cv2.imread('deneme2.png')


    img = cv2.imread(name)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([90, 0, 0])
    upper_blue = np.array([128, 255, 255])

    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    res = cv2.bitwise_and(img, img, mask=mask)

    cv2.imwrite("deneme3.png",res)
    img_blue = cv2.imread('deneme3.png')



sum_filter("img_1.png")