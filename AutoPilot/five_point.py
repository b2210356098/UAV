from dronekit import Command, connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

drone = connect("127.0.0.1:14550", wait_ready=False)
home_x=drone.location.global_frame.lat
home_y=drone.location.global_frame.lon
altitude = 5

def is_arrive(x,y):
    while not( x*0.9<drone.location.global_frame.lat<x*1.1) and not(y*0.9< drone.location.global_frame.lon <y*1.1):
        print("Going to the location...")
        time.sleep(0.5)

    

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
    print("Drone is rising")
    
    while drone.location.global_relative_frame.alt < height * 0.9:
        time.sleep(1)


def add_mission():
    global drone_command
    drone_command = drone.commands

    drone_command.clear()
    time.sleep(1)

    # TAKEOFF
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, altitude))

    # WAYPOINT
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,  -35.36316831 ,149.16523378 , altitude))
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,-35.36318643, 149.16541526, altitude))
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, -35.36326194 ,149.16539119, altitude))
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, home_x ,home_y, altitude))
    

    # VERIFICATION
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,-35.36321863 ,149.16559213, altitude))

    drone_command.upload()
    print("Commands are loading...")




def drop_ball(x,y):
    global drone_command
    drone_command = drone.commands

    drone_command.clear()
    time.sleep(1)

   
    # WAYPOINT
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, x,y, altitude))

    # VERIFICATION
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, x,y, altitude))

    drone_command.upload()
    print("Commands are loading...")

   

try:
    takeoff(10)

    add_mission()

    drone_command.next = 0

    drone.mode = VehicleMode("AUTO")# Necessary for load commands


    way_point =0


    while True:
        next_waypoint = drone_command.next

        if next_waypoint== way_point:
            way_point+=1
            print("Current Command : ",next_waypoint)

        if next_waypoint ==5 :
            break

    drop_ball( -35.36322074 ,149.16529242)

    drone_command.next = 0
    print(drone.mode)
    while True:
        next_waypoint = drone_command.next

        if next_waypoint ==0:
            print("Flying through the person...")
            time.sleep(2)
        elif next_waypoint ==1:
            print("Returning home ")
            break
    

finally:

    drone.mode = VehicleMode("RTL")
    print("Drone has returned the launch ")
