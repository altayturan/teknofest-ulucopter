from re import T
from dronekit import LocationGlobal, LocationGlobalRelative, connect,VehicleMode
import time
import socket
import argparse


connection_string="127.0.0.1:14550"
iha = connect(connection_string,wait_ready=True,timeout=50)
def arm(value):
    while iha.is_armable != True:
        time.sleep(1)
    iha.armed = value
def mode(mode):
    if iha.mode != mode:
        iha.mode = VehicleMode(mode)
def takeoff(alt):
    iha.simple_takeoff(alt)


mode("GUIDED")
arm(True)
takeoff(20)
time.sleep(15)
if iha.location.global_relative_frame.alt>=20*0.8:
    mode("LAND")
arm(False)

