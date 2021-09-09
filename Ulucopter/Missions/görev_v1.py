# Belirli bir kordinata gidip başladığı yere geri dönüyor.
import dronekit as dk
import time


######################### CONNECTION ##########################
connection_string = "127.0.0.1:14550"
iha = dk.connect(connection_string,wait_ready=True)

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
def git(wp,gs=100):                       #Ground speed rastgele verildi düzenlenecek
    iha.simple_goto(wp,groundspeed = gs)
def konum():
    wp = iha.location.global_relative_frame
    return wp

mode("GUIDED")
arm()
takeoff(10)
while iha.location.global_relative_frame.alt <= 10*0.95:
    time.sleep(1)
wp1 = konum()
wp2 = dk.LocationGlobalRelative(-35.36272587,149.16515331,10)
git(wp2)
time.sleep(2)
while iha.groundspeed >= 1:
    time.sleep(1)
git(wp1)
time.sleep(2)
while iha.groundspeed >= 1:
    time.sleep(1)
mode("LAND")
while iha.location.global_relative_frame.alt >=0.2:
    time.sleep(1)
mode("STABILIZE")
disarm()
    
            
