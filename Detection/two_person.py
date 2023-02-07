import os.path
import time
import cv2


x_center= 0
y_center = 0
cap = cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)


def make_calculation(x,y):


    cam_area = 1
    real_area= 10


    real_x_distance = (x-255)*real_area/cam_area
    real_y_distnance= (y-255)*real_area/cam_area

    x_y=list()
    x_y.append(real_x_distance/1000)
    x_y.append(real_y_distnance/1000)
    return x_y

classNames=["person"]

configPath ="ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightPath ="frozen_inference_graph.pb"

net = cv2.dnn_DetectionModel(weightPath,configPath)
net.setInputSize(320,320)
net.setInputScale(1.0/127.5)
net.setInputMean((127.5,127.5,127.5))
net.setInputSwapRB(True)
i=0
founded = False
while True:
    try:
        success,img = cap.read()

        classIds,confs,bbox=net.detect(img,confThreshold = 0.5)

        print(classIds)
        if len(classIds)!=0:

            print(i, "-> It found a person ")
            i += 1

            if i >= 5 and not founded:
                cv2.imwrite("first_person.png", img)
                founded = True
                time.sleep(5)
                i = 0

            elif i >= 5 and founded:
                cv2.imwrite("second_person.png", img)
                founded = True


            for classId,confidence ,box in zip(classIds.flatten(),confs.flatten(),bbox):
                #cv2.rectangle(img,box,(0,255,0),thickness= 2)
                #cv2.putText(img,classNames[classId-1].upper(),(box[0],box[1]),cv2.FONT_HERSHEY_COMPLEX,2,(0,255,0),2)
                #print("Xcor : ",(box[0] + box[2] )/2 , ", Ycor : ",(box[1]+ box[3])/2)
                x_center=(box[0] + box[2] )/2
                y_center = (box[1] + box[3] )/2

            if i>=5 and founded:
                break

        #cv2.imshow("Output",img)
        #cv2.waitKey(1)

    except :
        pass

print(x_center)
print(y_center)
print(make_calculation(x_center,y_center))
