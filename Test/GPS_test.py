import time
import cv2
import math
import haversine as hs
from dronekit import connect

import serial
import RPi.GPIO as GPIO



# Connection
#plane = connect("/dev/ttyACM0", wait_ready=False)# usb
plane = connect("/dev/serial0", wait_ready=False)#telemetry 

while (True):
    print("Waiting For GPS")
    time.sleep(0.1)
    if(plane.location.global_frame.lat!=None):
          break


print("Armable : ",plane.is_armable)
print("Armed : ",(plane.armed)
print("Lat : ",plane.location.global_frame.lat)
print("Lon : ",plane.location.global_frame.lon)
print("Alt : ",plane.location.global_frame.alt)
