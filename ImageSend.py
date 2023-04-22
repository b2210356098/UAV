import socket
import time
import cv2


server = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
#print("SERVER")
server.bind(("", 5297))
#print("BIND")
server.listen(5)
#print("LISTEN")
cap = cv2.VideoCapture(0)
"""
cap.set(3,320) # width is id no 3
cap.set(4,240) # height is id no 4
cap.set(10,100) # Brightness id is 10
"""
#print("VIDEO CAPTURE")
clientsocket, address = server.accept()
#print("BAGLAN")

while True:
    success, img = cap.read()
    #print("VIDEO READ")
    cv2.imwrite("frame.jpeg", img, [cv2.IMWRITE_JPEG_QUALITY, 50])
    #print("WRITE FRAME")
    with open("frame.jpeg","rb") as image:
        read_image = image.read()
        #print("READ IMAGE")
        byteimg = bytearray(read_image)
        #print("CONVERT TO BYTE")
    clientsocket.send(byteimg)
    #print("SEND")
