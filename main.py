from dronekit import Command, connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
#import cv2
import math


plane = connect("127.0.0.1:14550", wait_ready=False)

home_x = plane.location.global_frame.lat
home_y = plane.location.global_frame.lon

altitude = 15

gps_loc = [[-35.36291924, 149.16524853],
           [-35.36291127, 149.16568725],
           [-35.36322241, 149.16564782]]

x_drop = -35.36355641
y_drop = 149.16566748

i=0

x_center= 0
y_center = 0
"""

classNames=["person"]

configPath ="ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightPath ="frozen_inference_graph.pb"

net = cv2.dnn_DetectionModel(weightPath,configPath)
net.setInputSize(320,320)
net.setInputScale(1.0/127.5)
net.setInputMean((127.5,127.5,127.5))
net.setInputSwapRB(True)


cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)
"""

def gps_calculation(x1,y1,x2,y2,shapes):
    
    x3 =0
    y3 = 0
    return [x3,y3]

def takeoff(height):
    if plane.location.global_relative_frame.alt > height * 0.85:
        return
                
    while plane.is_armable is not True:
        print("Drone is not armable")
        time.sleep(1)
        pass


    print("Drone is armable")

    plane.mode = VehicleMode("GUIDED")

    plane.armed = True

    while plane.armed is not True:
        print("Drone is arming...")
        time.sleep(0.5)
        pass

    print("Drone has just armed")

    plane.simple_takeoff(height)

    while plane.location.global_relative_frame.alt < height * 0.9:
        print("Drone is rising")
        time.sleep(1)
        pass

    plane.mode = VehicleMode("AUTO")

def add_mission():
    global plane_command
    plane_command = plane.commands

    plane_command.clear()
    time.sleep(1)

    # TAKEOFF
    plane_command.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0,
                0, 0, 0, 0, altitude))

    # WAYPOINT
    plane_command.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0,
                0, 0, 0, gps_loc[0][0], gps_loc[0][1], altitude))
    plane_command.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0,
                0, 0, 0, gps_loc[1][0], gps_loc[1][1], altitude))
    plane_command.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0,
                0, 0, 0, gps_loc[2][0], gps_loc[2][1], altitude))

    # RETURN TO HOME
    plane_command.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0,
                0, 0, 0, home_x, home_y, altitude))

    # VERIFICATION
    plane_command.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0,
                0, 0, 0, 0, 0, 0, 0, 0))

    plane_command.upload()
    print("Commands are loading...")

def drop_ball(x, y):
    global plane_command
    plane_command = plane.commands

    plane_command.clear()
    time.sleep(1)

    # TAKEOFF
    plane_command.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0,
                0, 0, 0, 0, altitude))

    # WAYPOINT
    plane_command.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0,
                0, 0, 0, x, y, altitude))

    # RETURN TO HOME
    plane_command.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0,
                0, 0, 0, 0, 0, 0, 0, 0))

    # VERIFICATION
    plane_command.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0,
                0, 0, 0, 0, 0, 0, 0, 0))

    plane_command.upload()
    print("Commands are loading...")

founded_time = 0 #Hedefin bulunduğu zaman
wait_time =2 # İlk fotoğrafı çekip konum aldıktan sonra ikinci konum alma için geçmesini istediğimiz süre

first_x=0 # Fotoğraf çekildği andaki x konumu
first_y=0 # Fotoğraf çekildği andaki y konumu
second_x=0 # Fotoğraf çekildği andan wait_time kadar saniye sonraki x konumu
second_y=0 # Fotoğraf çekildği andan wait_time kadar saniye sonraki x konumu

gps_taked = False
photo_founded =False

shapes = [500,500]

try:

    takeoff(altitude)

    add_mission()

    plane_command.next = 0

    plane.mode = VehicleMode("AUTO")  # Necessary for load commands

    checker = 0
    while True:
        next_waypoint = plane_command.next
        if checker == next_waypoint:
            #Person Detection
            """
            try:
                # If it takes photo and time difference is bigger than 2 take second coordinates and continue
                if photo_founded:
                    if not gps_taked:
                        if time.time()-founded_time>=wait_time:
                            second_x = plane.location.global_frame.lat
                            second_y = plane.location.global_frame.lon
                            gps_taked = True

                    continue

                success, img = cap.read()
                shapes = img.shape

                classIds,confs,bbox=net.detect(img,confThreshold = 0.5)

                if len(classIds)!=0:

                    #print(i, "-> It found a person ")
                    i += 1

                    if i >= 5 :
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

            """
            
        else:
            checker = next_waypoint
            print(f"Next command : {next_waypoint}")


        if next_waypoint == 5:
            print("First Tour Completed")
            break
    #x_drop = gps_calculation(first_x,first_y,second_x,second_y,shapes)[0]
    #y_drop = gps_calculation(first_x,first_y,second_x,second_y,shapes)[1]

    drop_ball(x_drop, y_drop)
    plane_command.next = 0

    plane.mode = VehicleMode("AUTO")  # Görev yüklemek icin  gerekli
    while True:
        next_waypoint = plane_command.next

        print(f"Next command : {next_waypoint}")
        time.sleep(1)
        if next_waypoint == 3:
            print("Landing...")
            pass
        if next_waypoint == 4:
            print("Mission Completed")
            break
#finally:
except:
    while True:
        if  plane.location.global_relative_frame.alt < altitude * 0.85:
            takeoff(altitude)
        else:
            plane.mode = VehicleMode("CIRCLE")






