import time
import cv2

cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

classNames = ["person"]

configPath = "ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightPath = "frozen_inference_graph.pb"

net = cv2.dnn_DetectionModel(weightPath, configPath)
net.setInputSize(320, 320)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)
i = 0
printer_time = time.time()
printer = 0
while True:
    try:
        success, img = cap.read()

        classIds, confs, bbox = net.detect(img, confThreshold=0.5)

        if time.time() - printer_time >= 0.3:
            printer_time = time.time()
            cv2.imwrite(str(printer) + ".png", img)
            printer += 1
        if len(classIds) != 0:
            print(i, "-> Found a person with" + str(confs)+ " confidence ")
            i += 1

            for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
                cv2.rectangle(img, box, color=(0, 255, 0), thickness=2)
                cv2.putText(img, classNames[classId - 1].upper() + str(confs).upper(), (box[0], box[1]), cv2.FONT_HERSHEY_COMPLEX, 2,
                            (0, 255, 0), 2)
            cv2.imwrite(str(i) +"_"+ str(confs)+ "_person.png", img)
            i+=1

        cv2.imshow("Output", img)
        cv2.waitKey(1)
    except:
        pass
