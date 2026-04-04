import sys
import math
import serial
import time
import pynmea2
from geopy.distance import geodesic
import keyboard

def calculer_distance_et_cap(lat1, lon1, lat2, lon2):
    # Convertir les degrés en radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    # Différence de longitude
    dlon = lon2_rad - lon1_rad

    # Formule du cap initial
    y = math.sin(dlon) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
    cap = math.degrees(math.atan2(y, x))

    # Normaliser le cap entre 0° et 360°
    cap = (cap + 360) % 360
    # Calcul de la distance
    distance = geodesic((lat1, lon1), (lat2, lon2)).meters
    return distance, cap

def lire_coordonnees_gps(ser):
    while True:
        try:
            data = ser.readline().decode('ascii', errors='replace')
            if data.startswith('$GPGGA'):
                msg = pynmea2.parse(data)
                return msg.latitude, msg.longitude
        except:
            continue

#------------------------------MAIN-------------------------------

ser = serial.Serial(port="/dev/ttyACM0", baudrate=57600, timeout=0.1)
distance=0.0
cap=0.0

lat_cible = 47.391360   # Latitude cible (ex: Angers)
lon_cible = -0.739161    # Longitude cible

while True:
    if keyboard.is_pressed("esc"):
        print("\nBoucle arrètée par l'utilisateur.")
        break

    lat_actuel, lon_actuel = lire_coordonnees_gps(ser)
    distance, cap = calculer_distance_et_cap(lat_actuel, lon_actuel, lat_cible, lon_cible)
    print(f"{distance:.2f},{cap:.2f}",flush=True)
    sys.stdout.flush() 
