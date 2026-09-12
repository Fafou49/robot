import sys
import math
import serial
import time
import pynmea2
from geopy.distance import geodesic
import keyboard
from multiprocessing import Process

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
        except KeyboardInterrupt:
            break




ser = serial.Serial(port="/dev/ttyACM0", baudrate=57600, timeout=0.1)
# Filtre passe bas sur les données distance et cap.
#équivaut a une moyenne flottante

FENETRE = 5

# Initialisation des buffers circulaires
distances = []
caps = []
distance=0.0
cap=0.0
lat_cible = 47.391534   # Latitude cible (ex: Angers)
lon_cible = -0.739006   # Longitude cible

def my_loop():
    while True:
        lat_actuel, lon_actuel = lire_coordonnees_gps(ser)
        distance, cap = calculer_distance_et_cap(lat_actuel, lon_actuel, lat_cible, lon_cible)
        
        distances.append(distance)
        caps.append(cap)
        if len(distances) > FENETRE:
            distances.pop(0)
        if len(caps) > FENETRE:
            caps.pop(0)
        # Moyenne flottante sur la distance (simple moyenne arithmétique)
        distance_moy = sum(distances) / len(distances)
        
        # Moyenne flottante sur le cap (moyenne circulaire 0-360°)
        rads = [math.radians(c) for c in caps]
        sin_moy = sum(math.sin(r) for r in rads) / len(rads)
        cos_moy = sum(math.cos(r) for r in rads) / len(rads)
        cap_moy = (math.degrees(math.atan2(sin_moy, cos_moy)) + 360) % 360
        
        #print(f"{distances},{caps}",flush=True)
        print(f"{distance_moy:.2f},{cap_moy:.2f}",flush=True)
        sys.stdout.flush()

            
#------------------------------MAIN-------------------------------        
if __name__ == '__main__':  
    process = Process(target=my_loop)
    process.start()
    try :
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        ser.close()
        process.join()
        time.sleep(0.5)
        print("\nGPS TRANSFERT arrèté par l'utilisateur.")
        sys.exit(0)
        
