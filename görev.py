import dronekit as dk
import time
import cv2
from dronekit import mavlink
import numpy as np
from pymavlink import mavutil
######################### CONNECTION git##########################
connection_string = "/dev/ttyUSB0"  #/dev/serial0    baud = 921600
iha = dk.connect(connection_string,wait_ready=True,baud = 921600)

baslangic = dk.LocationGlobalRelative(40.2320732,28.8723660,7)
havuz =  dk.LocationGlobalRelative(40.2319631,23.8725360,7)
atesbaslangic =  dk.LocationGlobalRelative(40.2319660,28.8727680,7)
atesbitis = dk.LocationGlobalRelative(40.2321417,28.8728012,7)


####################### VARIABLES #############################
ortalamapayi = 60
suseviye = 2
ucusseviye = 7
atesseviye = 2

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

def mode(mode):
    iha.mode = dk.VehicleMode(mode)
    while mode != iha.mode:
        print(f"Mod değiştiriliyor. {mode} ")
        time.sleep(1)

def git(wp,gs=10):                       
    iha.simple_goto(wp,groundspeed = gs)
    print("Waypointe doğru gidiliyor.")


def yukseklik(alt):
    mode("GUIDED")
    iha.simple_goto((dk.LocationGlobalRelative(iha.location.global_relative_frame.lat,iha.location.global_relative_frame.lon,alt)))
    if alt<iha.location.global_relative_frame.alt:
        print("İnis yapılıyor.")
        while iha.location.global_relative_frame.alt > alt*1.15:
            time.sleep(1)
    
    elif alt>iha.location.global_relative_frame.alt:
        print("Kalkış yapılıyor.")
        while iha.location.global_relative_frame.alt < alt*0.85:
            time.sleep(1)
    
    print("Hedef yüksekliğe ulaşıldı.")

def send_velocity(velocity_x, velocity_y, velocity_z, duration=0):
    msg = iha.message_factory.set_position_target_local_ned_encode(0,0, 0, 
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        velocity_x, velocity_y, velocity_z, # m/s
        0, 0, 0,
        0, 0)
    
    for x in range(0,duration):
        iha.send_mavlink(msg)
        time.sleep(1)

def kirmiziAlgila():
    camera = cv2.VideoCapture(0)
    
    width = 640
    height = 480
    camera.set(3, width)
    camera.set(4, height)
    
    while True:
        _,frame = camera.read()
        
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        kernal = np.ones((5,5),"uint8")
        
        low_red = np.array([150, 40, 180])
        high_red = np.array([170, 255, 255])
        
        red_mask = cv2.inRange(hsv, low_red, high_red)
        dilated_redmask = cv2.dilate(red_mask, kernal)
        blured_redmask = cv2.GaussianBlur(dilated_redmask, (5,5), 4/6)
        
        contours_red, hierarchy = cv2.findContours(blured_redmask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        i=0
        
        for pic, contour in enumerate(contours_red):
            area = cv2.contourArea(contour)
            if area > 5000 :
                mode("BRAKE")
                x, y, w, h = cv2.boundingRect(contour)
                centerX = x+(w//2)
                centerY = y+(h//2)
                i+=1
                print("Su bırakma alanı algılandı")
                if i == 2:             
                    break
                camera.release()
                
                return centerX,centerY,width,height



def goDOGU(süre):
    send_velocity(0,0.35,0,süre)                             # BURADAKİ 2 M/S HIZ ÇOK FAZLADIR SİMİLASYON İÇİN YAPILMIŞTIR LÜTFEN UYGUN DEĞERLER GİRİNİZ VE İŞARETLERİ
                                                          # DEĞİŞTİRMEYİNİZ.
def goBATI(süre):
    send_velocity(0,-0.35,0,süre)

def goKUZEY(süre):
    send_velocity(0.35,0,0,süre)

def goGUNEY(süre):
    send_velocity(-0.35,0,0,süre)

def ortala(centerX,centerY,width,height):
    mode("GUIDED")
    
    is_ortalandi = False
    
    if centerX < width/2-ortalamapayi:
        print("Solda")
        goBATI(1)
        return is_ortalandi
    
    if centerX > width/2+ortalamapayi:
        print("Sagda")
        goDOGU(1)
        return is_ortalandi
    
    if centerY > height/2+ortalamapayi:
        print("Aşağıda")
        goGUNEY(1)
        return is_ortalandi
    
    if centerY < height/2-ortalamapayi:
        print("Yukarda")
        goKUZEY(1)
        return is_ortalandi
    
    if centerX > width/2-ortalamapayi and centerX < width/2+ortalamapayi and centerY < height/2+ortalamapayi and centerY > height/2-ortalamapayi:
        print("ortalandı")
        is_ortalandi = True
        return is_ortalandi

def suAlma():
    print("Su alınıyor.")
    #GPIO.setmode(GPIO.BCM) 
    #GPIO.setup(12, GPIO.OUT) 
    #GPIO.output(12,GPIO.HIGH) 
    time.sleep(5) 
    #GPIO.output(12,GPIO.LOW)
    #GPIO.cleanup()
    print("Su alındı")

def suBirakma():
    print("Su bırakılıyor.")
    # msg = iha.message_factory.command_long_encode(
    #         0, 0,   
    #         mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
    #         0, 
    #         13,            # servo number
    #         1000,          # servo position between 1000 and 2000
    #         0, 0, 0, 0, 0)

    # iha.send_mavlink(msg)
    time.sleep(3)
    print("Su bırakıldı.")
    # msg = iha.message_factory.command_long_encode(
    #         0, 0,    
    #         mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 
    #         0, 
    #         13,            # servo number
    #         1500,          # servo position between 1000 and 2000
    #         0, 0, 0, 0, 0)    

     
    # iha.send_mavlink(msg)

########################### MISSION ##############################
cmds = iha.commands

cmds.download()
cmds.wait_ready()
cmds.clear() #
cmd1 = dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, 1 #HIZ
                                                                                                                ,2, 0, 0, 0, 0,0)
cmd2 = dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 7,havuz.lat,havuz.lon,ucusseviye)
cmd3 = dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 7,atesbaslangic.lat,atesbaslangic.lon,ucusseviye)
cmd4 = dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 7,atesbitis.lat,atesbitis.lon,ucusseviye)
cmd5 = dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 7,baslangic.lat,baslangic.lon,ucusseviye)

cmds.add(cmd1)
cmds.add(cmd2)
cmds.add(cmd3)
cmds.add(cmd4)
cmds.add(cmd5)

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
        if nextwaypoint == 3:
            mode("BRAKE")
            time.sleep(1)
            yukseklik(2)
            suAlma()
            yukseklik(7)
            iha.commands.next = 4
            mode("AUTO")
        if nextwaypoint == 4:    
            is_ortalandi=False

            while is_ortalandi==False:
                centerX2,centerY2,width2,height2 = kirmiziAlgila()
                is_ortalandi=ortala(centerX2,centerY2,width2,height2)
                time.sleep(1)
            yukseklik(2)
            suBirakma()
            yukseklik(7)
            iha.commands.next = 5
            mode("AUTO")
        if nextwaypoint == 5:
            mode("RTL")
except Exception as e:
    print(e)
    mode("LAND")