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



    while True:
        next_waypoint = drone_command.next

        print(f"Next command : {next_waypoint}")
        time.sleep(1)

        if next_waypoint is 5:
            print("First Tour Completed")
            break


    drop_ball(x_drop,y_drop)
    drone_command.next = 0

    drone.mode = VehicleMode("AUTO")# Necessary for load commands
    while True:
        next_waypoint = drone_command.next

        print(f"Next command : {next_waypoint}")
        time.sleep(1)
        if next_waypoint is 3:
            print("Landing...")
        if next_waypoint is 4:
            print("Mission Completed")
            break
        
except:
    print("An error occured")

finally:
    drone.mode = VehicleMode("RTL")
    print("Drone Has Returned to Home")


   
