import dronekit as dk
import time

connection_string = "/dev/ttyUSB0"  #/dev/serial0    baud = 921600
iha = dk.connect(connection_string,wait_ready=True,baud=921600)

havuz = dk.LocationGlobalRelative(0,0,10)
baslangic = dk.LocationGlobalRelative(0,0,10)

def takeoff(alt):
    iha.simple_takeoff(alt)
    print("Takeoff yapılıyor.")
    while iha.location.global_relative_frame.alt <= alt*0.95:
        time.sleep(1)
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

def konum():
    time.sleep(5)
    wp = iha.location.global_relative_frame
    print("Konum alındı.")
    return wp

def yukseklik(alt):
    iha.simple_goto((dk.LocationGlobalRelative(iha.location.global_relative_frame.lat,iha.location.global_relative_frame.lon,alt)))
    if alt<iha.location.global_relative_frame.alt:
        print("İnis yapılıyor.")
        
    elif alt>iha.location.global_relative_frame.alt:
        print("Kalkış yapılıyor.")
    
    while iha.location.global_relative_frame.alt > alt*1.05:
        time.sleep(1)
    
    print("Hedef yüksekliğe ulaşıldı.")


mode("GUIDED")
arm()
takeoff(10)
git(havuz)
yukseklik(2)
time.sleep(10)
yukseklik(10)
git(baslangic)
mode("LAND")
