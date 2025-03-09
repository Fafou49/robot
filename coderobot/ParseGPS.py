import os
import time
import serial
import string
import pynmea2
from time import sleep

ser = serial.Serial(port="/dev/ttyACM0", baudrate=57600, timeout=0.1)

#$PMTK220,10000*2F
while True:
    dataout = pynmea2.NMEAStreamReader()
    newdata=ser.readline()
    if (newdata[0:6] == b"$GPRMC") and (newdata[17] == "A"):
        newmsg = pynmea2.parse(newdata.decode("utf-8"))
        lat=newmsg.latitude
        lng=newmsg.longitude
        gps = str(lat) + ", " + str(lng)
        print("found RMC "+ str(gps))
    if (newdata[0:6] == b"$GPGGA"):
        newmsg = pynmea2.parse(newdata.decode("utf-8"))
        lat=newmsg.latitude
        lng=newmsg.longitude
        gps = str(lat) + ", " + str(lng)
        print("found GGA "+ str(gps))
    sleep(0.1)