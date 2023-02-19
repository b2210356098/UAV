from dronekit import Command, connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import cv2
import math
import GPIO

# Connection
plane = connect("127.0.0.1:14550", wait_ready=False)

#Initialization
home_x = plane.location.global_frame.lat
home_y = plane.location.global_frame.lon

altitude = 25

#Waypoints
gps_loc = [[-35.36291924, 149.16524853],
           [-35.36291127, 149.16568725],
           [-35.36322241, 149.16564782]]

# Initialize person coordinates for worst case
person_gps_x, person_gps_y = (gps_loc[0][0] + gps_loc[1][0]) / 2, (gps_loc[0][1] + gps_loc[1][1]) / 2

# How many photo will take for verification 
photo_limit = 0

# Coordinates of person in photo
x_center = 0
y_center = 0
shapes = [600, 480] #Photo shapes initialization

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

    # Takes coordinates of photo and returns real distance between plane and person
    def distance_calculation(x, y, image):
        cam_area = 1
        real_area = 55
        ratio = real_area / cam_area

        # Makes center origin
        x = x - image.shape[1] / 2
        y = -y + image.shape[0] / 2

        # It makes coordinates between 0 and 1
        x = x / image.shape[1]
        y = y / image.shape[0]

        real_x = x * ratio
        real_y = y * ratio

        return [real_x, real_y]

    plane_dist_x, plane_dist_y = distance_calculation(photo_x, photo_y, image)

    # Finds angle between plane and geographic north point
    angle = math.atan((plane_lat2 - plane_lat) / (plane_lon2 - plane_lon)) * 57.2957795

    # Finds distance between person and plane  
    real_x = plane_dist_x * math.cos(angle) + plane_dist_y * math.sin(angle)
    real_y = -plane_dist_x * math.sin(angle) + plane_dist_y * math.cos(angle)

    # Finds real gps coordinates
    gps_x = plane_lat + (180 / math.pi) * (real_y / 6378137)
    gps_y = plane_lon + (180 / math.pi) * (real_x / 6378137) / math.cos(plane_lat * 57.2957795)


    return(gps_x, gps_y)


# It makes plane take off
def takeoff(height):
    # It checks if plane is already taken off
    if plane.location.global_relative_frame.alt > height * 0.85:
        return

    # If plane is not armable, this while loop works
    while plane.is_armable is not True:
        print("Drone is not armable")
        #time.sleep(1)
        pass

    print("Drone is armable")

    plane.mode = VehicleMode("GUIDED")

    plane.armed = True

    # It arms plane
    while plane.armed is not True:
        print("Drone is arming...")
        #time.sleep(0.5)
        pass

    print("Drone has just armed")

    # It takes off the plane
    plane.simple_takeoff(height)

    # It waits until plane arrive 0.85 off its destination
    while plane.location.global_relative_frame.alt < height * 0.85:
        print("Drone is rising")
        #time.sleep(1)
        pass

    # It makes plane modes AUTO
    plane.mode = VehicleMode("AUTO")

# It adds mission to the autopilot
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


# It flies to the target
def fly_to_the_person(x, y):
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

def drop_ball(person_gps_x, person_gps_y,current_x, current_y):
    if current_x > 4*(person_gps_x+home_x)/5 and current_y>4*(person_gps_y+home_y)/5:
        servo_control(1,0.5)


founded_time = 0  # The time, when the target is found
wait_time = 2  # The time we want to wait for taking second photo


first_x = 0  # x position when photo was taken
first_y = 0  # y position when photo was taken
second_x = 0  # x position after wait_time second from first_x
second_y = 0  # y position after wait_time second from first_y

gps_taken = False # Checks if second gps is taken or not
photo_founded = False # Checks if photo is founded or not


try:

    takeoff(altitude)

    add_mission()

    plane_command.next = 0 # Necessary for read load commands

    plane.mode = VehicleMode("AUTO")  # Necessary for load commands

    checker = 0 # Necessary for make process while flying
    while True:
        next_waypoint = plane_command.next # Reads commands from plane
        if checker == next_waypoint:
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


        else:
            checker = next_waypoint
            print(f"Next command : {next_waypoint}")

        if next_waypoint == 5:
            print("First Tour Completed")
            break

    person_gps_x = gps_calculation(first_x, first_y, second_x, second_y, shapes)[0]
    person_gps_y = gps_calculation(first_x, first_y, second_x, second_y, shapes)[1]

    fly_to_the_person(person_gps_x, person_gps_y)
    plane_command.next = 0 # Necessary for read load commands

    plane.mode = VehicleMode("AUTO")  # Necessary for load commands
    while True:
        next_waypoint = plane_command.next  # Reads commands from plane

        drop_ball(person_gps_x,person_gps_y,plane.location.global_frame.lat,plane.location.global_frame.lon)

        print(f"Next command : {next_waypoint}")
        #time.sleep(1)
        if next_waypoint == 3:
            print("Landing...")
            pass
        if next_waypoint == 4:
            print("Mission Completed")
            break


except:
    while True:
        if plane.location.global_relative_frame.alt < altitude * 0.85:
            takeoff(altitude)
        else:
            plane.mode = VehicleMode("CIRCLE")


