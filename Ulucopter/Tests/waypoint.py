from operator import mod
from re import T
import dronekit as dk
import time

connection_port="127.0.0.1:14550"
iha = dk.connect(connection_port,wait_ready=True,timeout=100)

def calis(yukseklik):
    while iha.is_armable==False:
        print("Arm edilemiyor")
        time.sleep(1)
    print("İha arm edilebilir")
    iha.mode=dk.VehicleMode("GUIDED")

    while iha.mode=="GUIDED":
        print("Guided moda geçiş yapılıyor.")
        time.sleep(1)
    print("GUIDED moda geçildi.")
    iha.armed=True
    while iha.armed is False:
        print("Arm bekleniyor.")
        time.sleep(1)
    print("İha arm oldu")
    iha.simple_takeoff(yukseklik)
    iha.location.global_relative_frame.alt=40
    while iha.location.global_relative_frame.alt <= yukseklik*0.94:
        print(f"Yukseklik: {iha.location.global_relative_frame.alt}")
        time.sleep(0.5)
    hedef = dk.LocationGlobalRelative(-35.36279451, 149.16516072,15)
    iha.simple_goto(hedef)
    


calis(20)