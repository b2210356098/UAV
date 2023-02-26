from dronekit import Command, connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import cv2
import math
#import GPIO

# Connection
plane = connect("127.0.0.1:14550", wait_ready=False)

photo_limit = 0

x_center = 0 # x location in photograph
y_center = 0 # y location in photograph
shapes = [0, 0]

founded_time = 0  # The time, when the target is found
wait_time = 2  # The time we want to wait for taking second photo


first_x = 0  # x position when photo was taken
first_y = 0  # y position when photo was taken
second_x = 0  # x position after wait_time second from first_x
second_y = 0  # y position after wait_time second from first_y

gps_taken = False # Checks if second gps is taken or not
photo_founded = False # Checks if photo is founded or not
# Person Recognization Model

classNames=["person"]

configPath ="ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightPath ="frozen_inference_graph.pb"

net = cv2.dnn_DetectionModel(weightPath,configPath)
net.setInputSize(320,320)
net.setInputScale(1.0/127.5)
net.setInputMean((127.5,127.5,127.5))
net.setInputSwapRB(True)

# Starts Camera
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)


# It brings some plane parameters and return real gps coordinates of person
def gps_calculation(plane_lat,plane_lon,plane_lat2,plane_lon2,photo_x,photo_y,image):

    # Fotoğraftaki kordinatları alarak tespit edilen cismin uçağa olan gerçek uzaklığını döndüyor
    def distance_calculation(x, y, shape):
        yükseklik = 2500
        yükseklik_alan = 3070.434033764282 * yükseklik ** 2 / 158.5 ** 2  # cm2
        yükselikteki_birim = math.sqrt(yükseklik_alan / (640 * 480))  # Her bir birim uzunluğun reeldeki cm karşılığı

        # Merkezi orijin olarak alıyor
        x = x - shape[1] / 2
        y = -y + shape[0] / 2

        real_x = yükselikteki_birim * x
        real_y = yükselikteki_birim * y
        return [real_x, real_y]

    plane_dist_x, plane_dist_y = distance_calculation(photo_x, photo_y, image)

    # Uçağın uçtuğu yol ile  coğrafik kuzey noktası arasındaki açıyı buluyor
    try:
        angle = math.atan((plane_lat2 - plane_lat) / (plane_lon2 - plane_lon)) * 57.2957795
    except ZeroDivisionError:
        angle=89

    # Hedefin coğrafik kordinatlara göre uçağa olan uzaklığını buluyor
    real_x = plane_dist_x * math.cos(angle) + plane_dist_y * math.sin(angle)
    real_y = -plane_dist_x * math.sin(angle) + plane_dist_y * math.cos(angle)

    # Hedefin gps kordinatlarını buluyor
    gps_x = plane_lat + (180 / math.pi) * (real_y / 6378137)
    gps_y = plane_lon + (180 / math.pi) * (real_x / 6378137) / math.cos(plane_lat * 57.2957795)


    return(gps_x, gps_y)

def servo_control(repeat, sleep):
    """
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
      """
    pass

def drop_ball(person_gps_x, person_gps_y,current_x, current_y):
    m,k,A,g = 1,1,1,9.8

    v_lim = math.sqrt(m*g/(k*A))

    ball_drop_time = plane.location.global_frame.alt*2/(3*v_lim)

    drop_distance = velocity_of_plane*ball_drop_time

    if ((current_x-person_gps_x)**2 + (current_y-person_gps_y)**2) <= drop_distance**2:
        servo_control(1,0.5)

    """

    if current_x > 4*(person_gps_x+home_x)/5 and current_y>4*(person_gps_y+home_y)/5:
        servo_control(1,0.5)
    """




try:
    while True:
        # Person Detection
        try:
            # If person not detected
            if photo_founded:
                if not gps_taken:
                    # if time difference is bigger than wait_time, it takes second coordinates

                    if time.time()-founded_time>=wait_time:
                        second_x = plane.location.global_frame.lat
                        second_y = plane.location.global_frame.lon
                        gps_taken = True

            # If person not detected, it searches for person and takes its photo and coordinates
            else:

                success, img = cap.read()
                shapes = img.shape

                classIds,confs,bbox=net.detect(img,confThreshold = 0.5)

                if len(classIds)!=0:


                    photo_limit += 1

                    if photo_limit >= 5 :
                        photo_founded =True
                        cv2.imwrite("first_person.png", img)
                        founded_time=time.time()
                        first_x = plane.location.global_frame.lat
                        first_y = plane.location.global_frame.lon

                    for classId,confidence ,box in zip(classIds.flatten(),confs.flatten(),bbox):

                        x_center = ( box[0] + box[2])/2
                        y_center = ( box[1] + box[3])/2
        except :
            pass


    velocity_of_plane = math.sqrt((first_x-second_x)**2 + (first_y-second_y)**2)/wait_time

    person_gps_x = gps_calculation(first_x, first_y, second_x, second_y, shapes)[0]
    person_gps_y = gps_calculation(first_x, first_y, second_x, second_y, shapes)[1]

    drop_ball(person_gps_x,person_gps_y,plane.location.global_frame.lat,plane.location.global_frame.lon)




except:

    if plane.location.global_relative_frame.alt < altitude * 0.85:
        takeoff(altitude)
    else:
        plane.mode = VehicleMode("CIRCLE")






