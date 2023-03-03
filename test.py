import time
import cv2
import math
import haversine as hs
from dronekit import connect

import serial
import RPi.GPIO as GPIO




flight_time = 10 # minute
photo_limit = 5
wait_for_delete = 3 #second
wait_for_fly = 0.10 #second


# Connection
plane = connect("/dev/ttyACM0", wait_ready=False)

while (True):
    print("Waiting For GPS")
    if(plane.location.global_frame.lat!=None):
          break

print("Armable, armed, version, velocity, alt,lat,lon")
print(plane.is_armable)
print(plane.armed)
print(plane.version)
print(plane.velocity)
try:
    x,y,a = 0,0,0
    loc1 = (plane.location.global_frame.lat, plane.location.global_frame.lon)
    loc2 = (x,y)
   
    for t in range(100):
        a = 0
        if ((hs.haversine(loc1, loc2) * 1000)<10 and a<10):
            a+=1
            print("Lat Diff : ", plane.location.global_frame.lat - x, " Lon Diff : ",plane.location.global_frame.lon - y, " Alt Diff : ", plane.location.global_frame.alt - a)
            x = plane.location.global_frame.lat
            y = plane.location.global_frame.lon
            a = plane.location.global_frame.alt
            time.sleep(0.1)
        elif a >= 10:
            break
        else:
            a = 0

except Exception as e:
    print(e)



def servo_control(repeat, sleep, pvm, freq, x1, x2):

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pvm, GPIO.OUT)
    p = GPIO.PWM(pvm, freq)
    p.start(50)
    i = 0
    try:

        while i < repeat:
            p.ChangeDutyCycle(x1)
            time.sleep(sleep)
            p.ChangeDutyCycle(x2)
            time.sleep(sleep)
            i += 1
            return True
    except:
        p.stop()
        GPIO.cleanup()


cap = cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)



classNames=["person"]

configPath ="ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightPath ="frozen_inference_graph.pb"

net = cv2.dnn_DetectionModel(weightPath,configPath)
net.setInputSize(320,320)
net.setInputScale(1.0/127.5)
net.setInputMean((127.5,127.5,127.5))
net.setInputSwapRB(True)



i=0
last_time = time.time()
start_time = time.time()
first_x = 0
first_y = 0
photo_founded = False
while (time.time()-start_time<flight_time*60):
    try:
        if not photo_founded :
            success,img = cap.read()

            classIds,confs,bbox=net.detect(img,confThreshold = 0.5)


            if time.time()-last_time >= wait_for_delete:
                i = 0

            if len(classIds)!=0:
                last_time = time.time()

                for classId,confidence ,box in zip(classIds.flatten(),confs.flatten(),bbox):
                    img = cv2.rectangle(img, box, (0, 255, 0), thickness=2)
                    img = cv2.putText(img, classNames[classId - 1].upper()+str(confs), (box[0], box[1]), cv2.FONT_HERSHEY_COMPLEX, 2,(0, 255, 0), 2)

                i += 1
                cv2.imwrite(str(i)+".png",img)
                print(str(i)+ "-> Person Founded "+str(confs))


                if i>=photo_limit:
                    first_x = plane.location.global_frame.lat
                    first_y = plane.location.global_frame.lon
                    print("Person Location : ",first_x,first_y )
                    cv2.imwrite("mission.png",img)
                    photo_founded = True



            #cv2.imshow("Output",img)
            #cv2.waitKey(1)
        else:

            servo_control(1, 1, 33, 80, 5, 12.5)
            servo_control(1, 1, 32, 35, 12.5, 5)
            if servo_control(1, 1, 32, 35, 12.5, 5):
                break

    except Exception as e:
        print(e)

