from dronekit import Command, connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import os
import math
import cv2


plane = connect("127.0.0.1:14550", wait_ready=False)

home_x = plane.location.global_frame.lat
home_y = plane.location.global_frame.lon

altitude = 15

gps_loc = [[-35.36291924, 149.16524853],
           [-35.36291127, 149.16568725],
           [-35.36322241, 149.16564782]]




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
    plane_command.next = 0

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

def fly_to_person(x, y):

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

def gps_calculation(plane_lat, plane_lon, plane_lat2, plane_lon2, photo_x, photo_y, shape):

    # Fotoğraftaki kordinatları alarak tespit edilen cismin uçağa olan gerçek uzaklığını döndüyor
    def distance_calculation(x, y, shape):
        cam_area = 1
        real_area = 55
        ratio = math.sqrt(real_area / cam_area)

        # Merkezi orijin olarak alıyor
        x = x - shape[1] / 2
        y = -y + shape[0] / 2

        # Tekrardan image.shape'e böldük çünkü x ölçeğinin 0 ile 1 arasında olmasını sağlıyor
        x = x / shape[1]
        y = y / shape[0]

        real_x = x * ratio
        real_y = y * ratio

        return [real_x, real_y]

    plane_dist_x, plane_dist_y = distance_calculation(photo_x, photo_y, shape)

    # Uçağın uçtuğu yol ile  coğrafik kuzey noktası arasındaki açıyı buluyor
    try:
        angle = math.atan((plane_lat2 - plane_lat) / (plane_lon2 - plane_lon)) * 57.2957795

    except Exception as e:
        print(e)
        angle = 0
    # Hedefin coğrafik kordinatlara göre uçağa olan uzaklığını buluyor
    real_x = plane_dist_x * math.cos(angle) + plane_dist_y * math.sin(angle)
    real_y = -plane_dist_x * math.sin(angle) + plane_dist_y * math.cos(angle)

    # Hedefin gps kordinatlarını buluyor
    gps_x = plane_lat + (180 / math.pi) * (real_y / 6378137)
    gps_y = plane_lon + (180 / math.pi) * (real_x / 6378137) / math.cos(plane_lat * 57.2957795)


    return(gps_x, gps_y)



start = time.time()
pid = os.fork()
if pid > 0:
        
    try:

        takeoff(altitude)


        add_mission()
        plane_command.next = 0

        plane.mode = VehicleMode("AUTO")  # Necessary for load commands

        checker = 0
        while True:
            with open("current_gps.txt","w") as f:
                f.write(str(plane.location.global_frame.lat))
                f.write("\n")
                f.write(str(plane.location.global_frame.lon))

            next_waypoint = plane_command.next
            if checker != next_waypoint:
                checker = next_waypoint
                print(f"Next command : {next_waypoint}")
            

            if next_waypoint == 5:
                print("First Tour Completed")
                break

        
        f =open("gps_cor.txt","r")
        gps_list =f.read().split("\n")
        x_drop = float(gps_list[0])
        y_drop = float(gps_list[1])
        
        f.close()

        print(gps_list)

        plane_command.next = 0
          
        #x_drop,y_drop = -35.36301703, 149.16547480
        fly_to_person(x_drop, y_drop)

    

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
        
    except Exception as e:
        print(e)
        if  plane.location.global_relative_frame.alt < altitude * 0.85:
            takeoff(altitude)
     

    finally:
        plane.mode = VehicleMode("RTL")

    exit()

else:

    x_center = 0
    y_center = 0

    classNames=["person"]

    configPath ="ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
    weightPath ="frozen_inference_graph.pb"

    net = cv2.dnn_DetectionModel(weightPath,configPath)
    net.setInputSize(320,320)
    net.setInputScale(1.0/127.5)
    net.setInputMean((127.5,127.5,127.5))
    net.setInputSwapRB(True)
    i=0
        
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)


    founded_time = 0
    wait_time = 3
    first_x = 0
    first_y = 0
    second_x = 0
    second_y = 0
    gps_taked = False
    photo_founded = False

    shape = 0

    while True:
        with open("current_gps.txt","r") as f:
            if f.read() == None:
                continue

        #Person Detection
        try:
            # If it takes photo and time difference is bigger than wait_time take second coordinates and continue
            if photo_founded:
                if not gps_taked:
                    if time.time()-founded_time>=wait_time:

                        with open("current_gps.txt","r") as f:
                            a=f.read().split("\n")

                            second_x = float(a[0])
                            second_y = float(a[1])
                            gps_taked = True
                            print("SECOND GPS :",second_x,second_y)

                            break

                continue

            success, img = cap.read()

            classIds,confs,bbox=net.detect(img,confThreshold = 0.5)
            shape = img.shape
            if len(classIds)!=0:

                print(i+1, "-> It found a person ")
                i += 1

                if i >= 5 :
                   

                    with open("current_gps.txt","r") as f:
                        while True:

                            a=f.read().split("\n")
                            print(a)
                            if a !=['']:
                                first_x = float(a[0])
                                first_y = float(a[1])
                             
                                print("FIRST GPS :",first_x,first_y)
                                break


                    photo_founded =True
                    cv2.imwrite("target.png", img)
                    founded_time=time.time()

                for classId,confidence ,box in zip(classIds.flatten(),confs.flatten(),bbox):

                    x_center = ( box[0] + box[2])/2
                    y_center = ( box[1] + box[3])/2


        except Exception as e:
            print(e)
            pass


    print(first_x,first_y,second_x,second_y,x_center,y_center,shape )
    file = open("gps_cor.txt","w")     
    file.write(str(gps_calculation(first_x,first_y,second_x,second_y,x_center,y_center,shape)[0])+"\n")  
    file.write(str(gps_calculation(first_x,first_y,second_x,second_y,x_center,y_center,shape)[1]))  

    file.close()
    exit()
