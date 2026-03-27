import math
import serial
import pynmea2
import time
from geopy.distance import geodesic

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

port = "/dev/ttyACM0"  # À adapter selon ton port série
ser = serial.Serial(port, baudrate=57600, timeout=1)

def lire_coordonnees_gps():
    while True:
        try:
            data = ser.readline().decode('ascii', errors='replace')
            if data.startswith('$GNGGA'):
                msg = pynmea2.parse(data)
                return msg.latitude, msg.longitude
        except:
            continue
while True:
    lat_actuel, lon_actuel = lire_coordonnees_gps()

    lat_cible = 47.470026   # Latitude cible (ex: st Nicolas)
    lon_cible = -0.561586    # Longitude cible
    distance, cap = calculer_distance_et_cap(lat_actuel, lon_actuel, lat_cible, lon_cible)
    print(f"Distance : {distance:.2f} mètres, Cap : {cap:.2f}°")
    print(f"Cap (angle) : {cap:.2f} degrés")
    time.sleep(1/5)