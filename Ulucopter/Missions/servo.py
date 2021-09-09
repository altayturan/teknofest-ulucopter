from re import T
from sys import flags
import dronekit as dk
import time
import cv2
import numpy as np
from pymavlink import mavutil

######################### CONNECTION ##########################
connection_string = "/dev/serial0"  #/dev/serial0    baud = 921600
iha = dk.connect(connection_string,wait_ready=True,baud = 921600)




msg = iha.message_factory.command_long_encode(
            0, 0,    # target_system, target_component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, #command
            0, #confirmation
            13,    # servo number
            1000,          # servo position between 1000 and 2000
            0, 0, 0, 0, 0)    # param 3 ~ 7 not used

# send command to vehicle
iha.send_mavlink(msg)
time.sleep(4)
msg = iha.message_factory.command_long_encode(
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, #command
        0, #confirmation
        13,    # servo number
        1500,          # servo position between 1000 and 2000
        0, 0, 0, 0, 0)    # param 3 ~ 7 not used
# send command to vehicle
iha.send_mavlink(msg)