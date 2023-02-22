import time
import cv2
import os

name = "basicvideo1.mp4"

for i in os.listdir():
    if i.startswith("basicvideo"):
        name= i[:10]+str(int(i[10])+1)+".mp4"

cap= cv2.VideoCapture(0)

width= int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height= int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

writer= cv2.VideoWriter(name, cv2.VideoWriter_fourcc(*'DIVX'), 20, (width,height))

start =time.time()

while time.time()-start<10:
    ret,frame= cap.read()

    writer.write(frame)

    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
writer.release()
cv2.destroyAllWindows()
