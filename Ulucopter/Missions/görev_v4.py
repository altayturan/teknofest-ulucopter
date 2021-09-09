# Renk algılayınca durmak

import dronekit as dk
import time

import cv2
import numpy as np
######################### CONNECTION ##########################
connection_string = "127.0.0.1:14550"
iha = dk.connect(connection_string,wait_ready=True)
is_kirmizi = False
is_mavi = False
######################### FUNCS ###############################
def takeoff(alt):
    iha.simple_takeoff(alt)
def arm():
    while iha.is_armable != True:
        print("Arm edilemiyor.")
        time.sleep(1)
    print("Arm ediliyor.")
    iha.armed = True
def disarm():
    # Yükseklik ya da mod kontrolü eklenmesi gerekebilir.
    iha.armed = False
def mode(mode):
    iha.mode = dk.VehicleMode(mode)
    while mode != iha.mode:
        print("Mod değiştiriliyor.")
        time.sleep(1)
def git(wp,gs=10):                       #Ground speed rastgele verildi düzenlenecek
    iha.simple_goto(wp,groundspeed = gs)
def konum():
    wp = iha.location.global_relative_frame
    return wp
def kirmiziAlgila():
    global is_kirmizi
    camera = cv2.VideoCapture(0)
    while True:
        _,frame = camera.read()       
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        kernal = np.ones((5,5),"uint8")
        low_red = np.array([165, 177, 69])
        high_red = np.array([179, 255, 255])
        red_mask = cv2.inRange(hsv, low_red, high_red)
        dilated_redmask = cv2.dilate(red_mask, kernal)
        blured_redmask = cv2.GaussianBlur(dilated_redmask, (5,5), 4/6)
        contours_red, hierarchy = cv2.findContours(blured_redmask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        i=0
        for pic, contour in enumerate(contours_red):
            area = cv2.contourArea(contour)
            if area > 500 :
                is_kirmizi = True
                ates = konum()
                i+=1
                print("algı")
                if i == 2:             
                    break
                if is_kirmizi != False:
                    print(1)
                    return ates
def maviAlgila():
    global is_mavi
    camera = cv2.VideoCapture(0)
    while True:
        _,frame = camera.read()       
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        kernal = np.ones((5,5),"uint8")
        i=0
        low_blue = np.array([100, 135, 40])
        high_blue = np.array([115, 255, 255])
        blue_mask = cv2.inRange(hsv, low_blue, high_blue)
        dilated_bluemask = cv2.dilate(blue_mask, kernal)
        blured_bluemask = cv2.GaussianBlur(dilated_bluemask, (5,5), 4/6)
        contours_blue, hierarchy = cv2.findContours(blured_bluemask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for pic, contour in enumerate(contours_blue):
            area = cv2.contourArea(contour)
            if area > 500 :
                is_mavi  = True
                
                i+=1
                print("algı")
                if i == 2:             
                    break
                if is_mavi != False:
                    print(1)
                    ates = konum()
                    return ates
##########################################################
mode("GUIDED")
arm()
takeoff(10)
while iha.location.global_relative_frame.alt <= 10*0.95:
    time.sleep(1)
wp1 = konum()
wp2 = dk.LocationGlobalRelative(-35.36272587,149.16515331,10)
git(wp2,1)
time.sleep(1)
ates = maviAlgila()
mode("BRAKE")



            