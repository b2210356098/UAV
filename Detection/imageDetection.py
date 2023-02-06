import random

import cv2
cap = cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)


classNames=[]
classFile = "coco.names"
with open(classFile,"rt" )as f:
    classNames = f.read().rstrip("\n").split("\n")

configPath ="ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightPath ="frozen_inference_graph.pb"

net = cv2.dnn_DetectionModel(weightPath,configPath)
net.setInputSize(320,320)
net.setInputScale(1.0/127.5)
net.setInputMean((127.5,127.5,127.5))
net.setInputSwapRB(True)
i=0
while True:
    try:
        success,img = cap.read()

        classIds,confs,bbox=net.detect(img,confThreshold = 0.5)
        try:
            if(classIds == 1):
                print(i,"-> It found a person ")
                i+=1

        except ValueError:
            pass
        #print(classIds,bbox)

        if len(classIds)!=0:


            for classId,confidence ,box in zip(classIds.flatten(),confs.flatten(),bbox):
                cv2.rectangle(img,box,color =(0,255,0),thickness = 2)
                cv2.putText(img,classNames[classId-1].upper(),(box[0],box[1]),cv2.FONT_HERSHEY_COMPLEX,2,(0,255,0),2)



        cv2.imshow("Output",img)
        cv2.waitKey(1)
    except IndexError:
        pass
