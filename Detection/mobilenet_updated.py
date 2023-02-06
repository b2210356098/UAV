import cv2
import time
import RPi.GPIO as GPIO
import time

#repeat: servonun kaç kez 5-12.5 arasında gidip geleceği, sleep: gidip gelme arasında bekleyeceği sre
def servo_control(repeat, sleep):
  GPIO.setmode(GPIO.BOARD)
  GPIO.setup(33, GPIO.OUT)
  p = GPIO.PWM(33, 50)
  p.start(2.5)
  i=0
  try:

      while i<repeat :
        p.ChangeDutyCycle(5)
        time.sleep(sleep)
        p.ChangeDutyCycle(12.5)
        time.sleep(sleep)
        i+=1
  except :
      p.stop()
      GPIO.cleanup()


cap = cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)

#coco dataset'i kullan
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
i = 0



while True:
    try:
        success,img = cap.read()
        start=time.time()
        
        classIds,confs,bbox=net.detect(img,confThreshold = 0.5)
        try:
            print(classIds)
            if(1 in classIds):
                
                print(i," -> It found a person")
                i+=1
                #doğruluk oranını arttırmak içi 5 kez person detect etmesini sağlayarak
                #son detect photosunu kaydeder ve servo'yu çalıştırır
                if(i==5):
                    cv2.imwrite("person.png",img)
                    servo_control(3,0.5)
                    break
               
        except ValueError:
            pass
        #print(classIds,bbox)

        end=time.time()
        total_tme=end-start

        fps=1/total_tme

        #detect ettiği person'ı rectangle ile işaretler
        if len(classIds)!=0:
            for classId,confidence ,box in zip(classIds.flatten(),confs.flatten(),bbox):
                cv2.rectangle(img,box,color =(0,255,0),thickness = 2)
                cv2.putText(img,classNames[classId-1].upper(),(box[0],box[1]),cv2.FONT_HERSHEY_COMPLEX,2,(0,255,0),2)
        #fps'i sonuç ekranında gösterir       
        cv2.putText(img,f"FPS: {int(fps)}",(20,70),cv2.FONT_HERSHEY_SIMPLEX,1.5,(0,255,0),2)
       
        #sonuç ekranını oluşturur
        cv2.imshow("Output",img)
        
        #"q" ile programı sonlandırır
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    except IndexError:
        pass
