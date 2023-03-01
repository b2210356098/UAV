import random
import time

import cv2
cap = cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)

t_file = open("Thresholds.txt","w")


classNames=["person"]

configPath ="ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightPath ="frozen_inference_graph.pb"

net = cv2.dnn_DetectionModel(weightPath,configPath)
net.setInputSize(320,320)
net.setInputScale(1.0/127.5)
net.setInputMean((127.5,127.5,127.5))
net.setInputSwapRB(True)
i=1
while True:
    try:

        success,img = cap.read()


        classIds,confs,bbox=net.detect(img,confThreshold = 0.5)

        if len(classIds)!=0:

            for classId,confidence ,box in zip(classIds.flatten(),confs.flatten(),bbox):
                img = cv2.rectangle(img, box, (0, 255, 0), thickness=2)
                img = cv2.putText(img, classNames[classId - 1].upper()+str(confs), (box[0], box[1]), cv2.FONT_HERSHEY_COMPLEX, 2,(0, 255, 0), 2)

            t_file.write(str(i)+ "-> Person Founded "+str(confs)+"\n")
            print(str(i)+ "-> Person Founded "+str(confs))

            cv2.imwrite(str(i)+"_"+str(confs) + "_person.png", img)

            i += 1



        #cv2.imshow("Output",img)
        #cv2.waitKey(1)

    except IndexError:
        pass

t_file.close()
