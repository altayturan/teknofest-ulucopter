import dronekit as dk
import time
import cv2
import numpy as np
from pymavlink import mavutil

######################### CONNECTION ##########################
connection_string = "127.0.0.1:14550"  #/dev/serial0    baud = 921600
iha = dk.connect(connection_string,wait_ready=True)

######################### LOCATIONS  ##########################
pistbaslangic = dk.LocationGlobalRelative(-35.36339578,149.16527995,10)
pistbitis = dk.LocationGlobalRelative(-35.36271838 ,149.16526675,10)
turtamam2spline = dk.LocationGlobalRelative( -35.36252214,149.16502282,10)
atesöncesi = dk.LocationGlobalRelative(-35.36271569, 149.16477890,10)
atesbaslangic = dk.LocationGlobalRelative(-35.36271569 ,149.16477890,10)  
atesbitis = dk.LocationGlobalRelative( -35.36338235 ,149.16480198 ,10)
turtamam1spline = dk.LocationGlobalRelative( -35.36356783, 149.16505910 ,10)

havuz = dk.LocationGlobalRelative(-35.36262698 ,149.16486130 ,15)
ates = iha.location.global_relative_frame
####################### VARIABLES #############################
ortalamapayi = 100
suseviye = 5
ucusseviye = 10
atesseviye = 5

######################### FUNCS ###############################
def takeoff(alt):
    iha.simple_takeoff(alt)
    print("Takeoff yapılıyor.")
    iha.wait_for_alt(alt)
    print("Takeoff yapıldı.")

def arm():
    while iha.is_armable != True:
        print("Arm edilemiyor.")
        time.sleep(1)
    print("Arm ediliyor.")
    iha.armed = True

def disarm():
    iha.armed = False
    print("Disarm ediliyor.")

def mode(mode):
    iha.mode = dk.VehicleMode(mode)
    while mode != iha.mode:
        print(f"Mod değiştiriliyor. {mode} ")
        time.sleep(1)

def git(wp,gs=10):                       
    iha.simple_goto(wp,groundspeed = gs)
    print("Waypointe doğru gidiliyor.")

def konum():
    time.sleep(5)
    wp = iha.location.global_relative_frame
    print("Konum alındı.")
    return wp

def yükseklik(alt):
    iha.simple_goto((dk.LocationGlobalRelative(iha.location.global_relative_frame.lat,iha.location.global_relative_frame.lon,alt)))
    if alt<iha.location.global_relative_frame.alt:
        print("İnis yapılıyor.")
        
    elif alt>iha.location.global_relative_frame.alt:
        print("Kalkış yapılıyor.")
    iha.wait_for_alt(suseviye)
    print("Hedef yüksekliğe ulaşıldı.")

def send_velocity(velocity_x, velocity_y, velocity_z, duration=0):
    msg = iha.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
        0b0000111111000111, # type_mask
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # m/s
        0, 0, 0, # x, y, z acceleration
        0, 0)
    for x in range(0,duration):
        iha.send_mavlink(msg)
        time.sleep(1)

def kirmiziAlgila():
    camera = cv2.VideoCapture(0)
    width = 1280
    height = 720
    camera.set(3, width)
    camera.set(4, height)
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
            if area > 100 :
                mode("BRAKE")
                time.sleep(1)
                x, y, w, h = cv2.boundingRect(contour)
                centerX = x+(w//2)
                centerY = y+(h//2)
                i+=1
                print("Su bırakma alanı algılandı")
                if i == 2:             
                    break
                camera.release()
                return centerX,centerY,width,height

def maviAlgila():
    camera = cv2.VideoCapture(0)
    width = 1280
    height = 720
    camera.set(3, width)
    camera.set(4, height)
    while True:
        _,frame = camera.read()       
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        kernal = np.ones((5,5),"uint8")
        low_blue = np.array([100, 135, 40])
        high_blue = np.array([115, 255, 255])
        blue_mask = cv2.inRange(hsv, low_blue, high_blue)
        dilated_bluemask = cv2.dilate(blue_mask, kernal)
        blured_bluemask = cv2.GaussianBlur(dilated_bluemask, (5,5), 4/6)
        contours_blue, hierarchy = cv2.findContours(blured_bluemask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        i=0
        for pic, contour in enumerate(contours_blue):
            area = cv2.contourArea(contour)
            if area > 500 :
                x, y, w, h = cv2.boundingRect(contour)
                centerX = x+(w//2)
                centerY = y+(h//2)
                i+=1
                print("Su alma alanı algılandı")
                if i == 2:             
                    break
                camera.release()
                return centerX,centerY,width,height

def goDOGU(süre):
    send_velocity(0,2,0,süre)                             # BURADAKİ 2 M/S HIZ ÇOK FAZLADIR SİMİLASYON İÇİN YAPILMIŞTIR LÜTFEN UYGUN DEĞERLER GİRİNİZ VE İŞARETLERİ
                                                          # DEĞİŞTİRMEYİNİZ.
def goBATI(süre):
    send_velocity(0,-2,0,süre)

def goKUZEY(süre):
    send_velocity(2,0,0,süre)

def goGUNEY(süre):
    send_velocity(-2,0,0,süre)

def ortala(centerX,centerY,width,height):
    mode("GUIDED")
    
    is_ortalandi = False
    
    if centerX < width/2-ortalamapayi:
        print("Sağda")
        goDOGU(2)
        return is_ortalandi
    
    if centerX > width/2+ortalamapayi:
        print("Solda")
        goBATI(2)
        return is_ortalandi
    
    if centerY > height/2+ortalamapayi:
        print("Aşağıda")
        goKUZEY(2)
        return is_ortalandi
    
    if centerY < height/2-ortalamapayi:
        print("Yukarda")
        goGUNEY(2)
        return is_ortalandi
    
    if centerX > width/2-ortalamapayi and centerX < width/2+ortalamapayi and centerY < height/2+ortalamapayi and centerY > height/2-ortalamapayi:
        print("ortalandı")
        is_ortalandi = True
        return is_ortalandi

def suAlma():
    print("Su alınıyor.")
    time.sleep(5)
    print("Su alındı")

def suBırakma():
    print("Su bırakılıyor.")
    msg = iha.message_factory.command_long_encode(
            0, 0,    # target_system, target_component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, #command
            0, #confirmation
            13,    # servo number
            1000,          # servo position between 1000 and 2000
            0, 0, 0, 0, 0)    # param 3 ~ 7 not used

    # send command to vehicle
    iha.send_mavlink(msg)
    time.sleep(2)
    print("Su bırakıldı.")
    msg = iha.message_factory.command_long_encode(
            0, 0,    # target_system, target_component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, #command
            0, #confirmation
            13,    # servo number
            1500,          # servo position between 1000 and 2000
            0, 0, 0, 0, 0)    # param 3 ~ 7 not used

    # send command to vehicle
    iha.send_mavlink(msg)

########################### MISSION ##############################
cmds = iha.commands

cmds.download()
cmds.wait_ready()
cmds.clear() #
cmd1 = dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,pistbitis.lat,pistbitis.lon,ucusseviye)
cmd2 = dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT, 0, 0, 0, 0, 0, 0,turtamam2spline.lat,turtamam2spline.lon,ucusseviye)
cmd3 = dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT, 0, 0, 0, 0, 0, 0,atesöncesi.lat,atesöncesi.lon,ucusseviye)
cmd4 = dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, 2 #HIZ
                                                                                                                , -1, 0, 0, 0, 0,0)
cmd5 = dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,atesbaslangic.lat,atesbaslangic.lon,ucusseviye)
cmd6 = dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,atesbitis.lat,atesbitis.lon,ucusseviye)
cmd7 = dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, 10 #HIZ
                                                                                                                , -1, 0, 0, 0, 0,0)
cmd8 = dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT, 0, 0, 0, 0, 0, 0,turtamam1spline.lat,turtamam1spline.lon,ucusseviye)
cmd9 = dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT, 0, 0, 0, 0, 0, 0,pistbaslangic.lat,pistbaslangic.lon,ucusseviye)

cmd10 = dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,pistbitis.lat,pistbitis.lon,ucusseviye)
cmd11 = dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT, 0, 0, 0, 0, 0, 0,turtamam2spline.lat,turtamam2spline.lon,ucusseviye)
cmd12 = dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, 2 #HIZ
                                                                                                                , -1, 0, 0, 0, 0,0)
cmd13 = dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT, 0, 0, 0, 0, 0, 0,havuz.lat,havuz.lon,ucusseviye)
cmd14 = dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,atesbitis.lat,atesbitis.lon,ucusseviye)
cmd15 = dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, 10 #HIZ 
                                                                                                                    , -1, 0, 0, 0, 0,0)
cmd16 = dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT, 0, 0, 0, 0, 0, 0,turtamam1spline.lat,turtamam1spline.lon,ucusseviye)
cmd17 = dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT, 0, 0, 0, 0, 0, 0,pistbaslangic.lat,pistbaslangic.lon,ucusseviye)

cmds.add(cmd1)
cmds.add(cmd2)
cmds.add(cmd3)
cmds.add(cmd4)
cmds.add(cmd5)
cmds.add(cmd6)
cmds.add(cmd7)
cmds.add(cmd8)
cmds.add(cmd9)
cmds.add(cmd10)
cmds.add(cmd11)
cmds.add(cmd12)
cmds.add(cmd13)
cmds.add(cmd14)
cmds.add(cmd15)
cmds.add(cmd16)
cmds.add(cmd17)

cmds.upload() 
################################################################
try:
    mode("GUIDED")
    arm()
    takeoff(ucusseviye)

    iha.commands.next=0

    mode("AUTO")
    while True:
        nextwaypoint = iha.commands.next

        if nextwaypoint == 6:
            is_ortalandi=False
            
            while is_ortalandi==False:
                centerX,centerY,width,height = kirmiziAlgila()
                is_ortalandi=ortala(centerX,centerY,width,height)

            ates = konum()
            ates.alt = atesseviye
            iha.commands.next = 8
            mode("AUTO")

        if nextwaypoint == 13:
            mode("GUIDED")

            is_ortalandi=False

            while is_ortalandi==False:
                centerX,centerY,width,height = maviAlgila()
                is_ortalandi=ortala(centerX,centerY,width,height)

            yükseklik(suseviye)
            suAlma()
            git(ates)
            time.sleep()
            
            while iha.airspeed > 1:
                time.sleep(1)

            is_ortalandi=False

            while is_ortalandi==False:
                centerX,centerY,width,height = kirmiziAlgila()
                is_ortalandi=ortala(centerX,centerY,width,height)
            
            suBırakma()

            iha.commands.next = 14

            mode("AUTO")
            time.sleep(2)

            while iha.airspeed > 1:
                time.sleep(1)
            break

    mode("RTL")
except:
    mode("LAND")