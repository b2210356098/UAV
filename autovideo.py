from dronekit import Command, connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time


drone = connect("127.0.0.1:14550", wait_ready=False)

home_x=drone.location.global_frame.lat
home_y=drone.location.global_frame.lon

altitude = 25

gps_loc=[[ -35.36291924, 149.16524853],
        [-35.36291127 ,149.16568725],
        [ -35.36322241, 149.16564782]]

x_drop = -35.36355641
y_drop = 149.16566748
        
"""
x_center= 0
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
founded = False


cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)
"""


def takeoff(height):
    while drone.is_armable is not True:
        print("Drone is not armable")
        time.sleep(1)


    print("Drone is armable")

    drone.mode = VehicleMode("GUIDED")

    drone.armed = True

    while drone.armed is not True:
        print("Drone is arming...")
        time.sleep(0.5)

    print("Drone has just armed")

    drone.simple_takeoff(height)
    
    while drone.location.global_relative_frame.alt < height * 0.9:
        print("Drone is rising")
        time.sleep(1)

def add_mission():
    global drone_command
    drone_command = drone.commands

    drone_command.clear()
    time.sleep(1)

    # TAKEOFF
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, altitude))

    # WAYPOINT
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, gps_loc[0][0],  gps_loc[0][1], altitude))
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, gps_loc[1][0], gps_loc[1][1], altitude))
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,   gps_loc[2][0],  gps_loc[2][1], altitude))

    # RETURN TO HOME
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, home_x ,home_y, altitude))

    # VERIFICATION
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0))

    drone_command.upload()
    print("Commands are loading...")


def drop_ball(x,y):
    global drone_command
    drone_command = drone.commands
    
    
    drone_command.clear()
    time.sleep(1)

    # TAKEOFF
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, altitude))

    # WAYPOINT
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, x , y , altitude))

    # RETURN TO HOME
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    
    # VERIFICATION
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0))

    drone_command.upload()
    print("Commands are loading...")


try:
        
    takeoff(10)

    add_mission()

    drone_command.next = 0

    drone.mode = VehicleMode("AUTO")# Necessary for load commands


    checker=0
    while True:
        next_waypoint = drone_command.next
        if checker==next_waypoint:
            """
            #Person Detection
            try:
                success, img = cap.read()

                classIds,confs,bbox=net.detect(img,confThreshold = 0.5)

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
                   
                        x_center=(box[0] + box[2] )/2
                        y_center = (box[1] + box[3] )/2

                    if i>=5 and founded:
                        break

            except :
                pass
            """
            
            continue
        else:
            checker = next_waypoint
            print(f"Next command : {next_waypoint}")
        #time.sleep(1)

        if next_waypoint == 5:
            print("First Tour Completed")
            break

    drop_ball(x_drop,y_drop)
    drone_command.next = 0

    drone.mode = VehicleMode("AUTO")# Necessary for load commands
    while True:
        next_waypoint = drone_command.next

        print(f"Next command : {next_waypoint}")
        time.sleep(1)
        if next_waypoint == 3:
            print("Landing...")
        if next_waypoint == 4:
            print("Mission Completed")
            break

except:
    print("An error occured")



