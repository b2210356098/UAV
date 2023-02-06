import cv2
import numpy as np

import matplotlib.pyplot as plt

"""
camera = cv2.VideoCapture(0)
return_value, image = camera.read()
cv2.imwrite("deneme.png", image)

"""


def create_bar(height, width, color):
    bar = np.zeros((height, width, 3), np.uint8)
    bar[:] = color
    red, green, blue = int(color[2]), int(color[1]), int(color[0])
    return bar, (red, green, blue)

def color_list(name):
    img = cv2.imread(name)
    height, width, _ = np.shape(img)
    # print(height, width)

    data = np.reshape(img, (height * width, 3))
    data = np.float32(data)

    number_clusters = 5
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    flags = cv2.KMEANS_RANDOM_CENTERS
    compactness, labels, centers = cv2.kmeans(data, number_clusters, None, criteria, 10, flags)
    # print(centers)

    font = cv2.FONT_HERSHEY_SIMPLEX
    bars = []
    rgb_values = []

    for index, row in enumerate(centers):
        bar, rgb = create_bar(200, 200, row)
        bars.append(bar)
        rgb_values.append(rgb)

    img_bar = np.hstack(bars)
    c_list = []
    for index, row in enumerate(rgb_values):
        image = cv2.putText(img_bar, f'{index + 1}. RGB: {row}', (5 + 200 * index, 200 - 10),
                            font, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
        # print(f'{index + 1}. RGB{row}')
        c_list.append(row)
    """
    for i in c_list:

        if i[2] - i[0] > 50:
            # print("It is blue")
            return 0
        elif i[0] - i[2] > 50:
            # print("It is red")
            return 1   

    cv2.imshow('Image', img)
    cv2.imshow('Dominant colors', img_bar)

    # cv2.imwrite('output/bar.jpg', img_bar)

    cv2.waitKey(0)
    """

    return (c_list)

def red_filter(name):
    img = cv2.imread(name)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 50, 70])
    upper_red = np.array([9, 255, 255])

    mask = cv2.inRange(hsv, lower_red, upper_red)
    res = cv2.bitwise_and(img, img, mask=mask)

    cv2.imwrite("red.png", res)

def blue_filter(name):
    img = cv2.imread(name)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([90, 0, 0])
    upper_blue = np.array([128, 255, 255])

    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    res = cv2.bitwise_and(img, img, mask=mask)

    cv2.imwrite("blue.png", res)

def dominant(name):
    red_filter(name)
    blue_filter(name)

    print("RED LIST : ",color_list("red.png"))
    print("BLUE LIST : ",color_list("blue.png"))

    total_blue = 0
    total_red= 0
    for i in color_list("red.png"):
        total_red += i[0]
    for i in color_list("blue.png"):
        total_blue += i[2]
    print("TOTAL RED : ",total_red)
    print("TOTAL BLUE : ",total_blue)


    return 0 if total_red>total_blue else 1

print(dominant("img_1.png"))