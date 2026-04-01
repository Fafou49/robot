import sys
import datetime
import time
import math
import gpiod
import threading
import serial

class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp  # Coefficient proportionnel
        self.ki = ki  # Coefficient intégral
        self.kd = kd  # Coefficient dérivé
        self.setpoint = setpoint  # Valeur cible (distance ou angle)
        self.integral = 0
        self.previous_error = 0

    def update(self, measured_value, dt):
        error = self.setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output 
    
def move_to_target( current_distance, current_angle, left_speed, right_speed):
    dt = 1/5  # Pas de temps (en secondes)
    
    while True:
        # Mise à jour des PID
        
        speed = pid_distance.update(current_distance, dt)
        angular_velocity = pid_angle.update(current_angle, dt)
        
        # Limiter les valeurs de sortie si nécessaire
        speed = max(min(speed, 1.0), -1.0)*255
        angular_velocity = max(min(angular_velocity, 360), -360)

        # Simulation : mise à jour de la position (à remplacer par ton système rèel)
        #current_distance += speed * dt
        #current_angle += angular_velocity * dt
        
        # Conversion pour les moteurs (exemple pour roues différentielles)
        wheel_base = 0.59  # Distance entre les roues (en mètres)
        left_speed = speed - (angular_velocity * wheel_base / 2)
        left_speed = max(min(left_speed, 255), -255)
        right_speed = speed + (angular_velocity * wheel_base / 2)
        right_speed = max(min(right_speed, 255), -255)
        # Affichage (pour le débogage)
        print(f"Distance: {current_distance:.2f}, Angle: {current_angle:.2f} / left_speed :{left_speed:.2f}, right_speed :{right_speed:.2f}")

        # Condition d'arrêt
        if abs(pid_distance.setpoint - current_distance) < 0.1 and abs(pid_angle.setpoint - current_angle) < 0.1:
            print("Cible atteinte !")
            break

        time.sleep(dt)    

def pwm():
        while True:
            # 5.1 - Counting from 0 to 255 all the time
            for PWM_Counter in range(255):
                print("dutyCycleRight : ", right_speed, " / ", "dutyCycleLeft : ", left_speed)
                if dutyCycleLeft > 20:
                    if PWM_Counter > left_speed  :
                        M1_S1.set_value(0)
                    else:
                        M1_S1.set_value(1)
                elif dutyCycleLeft < -20:
                    if PWM_Counter > abs(left_speed):
                        M1_S2.set_value(1)
                else:
                    M1_S1.set_value(0)
                    M1_S2.set_value(0)

                if rigt_speed > 20:
                    if PWM_Counter > right_speed:
                        M2_S1.set_value(0)
                    else:
                        M2_S1.set_value(1)
                elif rigt_speed < -20:
                    if PWM_Counter > abs(right_speed):
                        M2_S2.set_value(0)
                    else:
                        M2_S2.set_value(1)
                else:
                    M2_S1.set_value(0)
                    M2_S2.set_value(0)


def lire_coordonnees_gps(ser):
    while True:
        try:
            data = ser.readline().decode('ascii', errors='replace')
            if data.startswith('$GPGGA'):
                msg = pynmea2.parse(data)
                return msg.latitude, msg.longitude
        except:
            continue
            
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

if __name__ == '__main__':

    left_speed=0
    right_speed=0
    lat_cible = 47.391360   # Latitude cible (ex: Angers)
    lon_cible = -0.739161    # Longitude cible
    ser = serial.Serial(port="/dev/ttyACM0", baudrate=57600, timeout=0.1)
    
    while True:
        lat_actuel, lon_actuel = lire_coordonnees_gps(ser)
        print(lat_actuel, lon_actuel)
        # ------------------------------------SETTINGS------------------------------------------------------
        chip = gpiod.Chip('gpiochip4')

        MOTOR1_SENS1 = 14
        MOTOR1_SENS2 = 15
        MOTOR2_SENS1 = 17
        MOTOR2_SENS2 = 27
                        
        M1_S1 = chip.get_line(MOTOR1_SENS1)
        M1_S1.request(consumer="M1_S1", type=gpiod.LINE_REQ_DIR_OUT)

        M1_S2 = chip.get_line(MOTOR1_SENS2)
        M1_S2.request(consumer="M1_S2", type=gpiod.LINE_REQ_DIR_OUT)

        M2_S1 = chip.get_line(MOTOR2_SENS1)
        M2_S1.request(consumer="M2_S1", type=gpiod.LINE_REQ_DIR_OUT)

        M2_S2 = chip.get_line(MOTOR2_SENS2)
        M2_S2.request(consumer="M2_S2", type=gpiod.LINE_REQ_DIR_OUT)



        Robot_width = 0.59  # change le width by the robot true width

        pid_distance = PIDController(kp=1.0, ki=0.1, kd=0.01, setpoint=0)
        pid_angle = PIDController(kp=0.5, ki=0.01, kd=0.1, setpoint=0)
# --------------------------------------------------------------------------------------------------
        distance, cap = calculer_distance_et_cap(lat_actuel, lon_actuel, lat_cible, lon_cible)
        try:
            while True:
                if keyboard.is_pressed("esc"):
                    print("\nBoucle arrètée par l'utilisateur.")
                    break
                for line in sys.stdin:
                    #print(line.strip().split(','))
                    Distance, cap =line.strip().split(',')
                    #print(f"{type(Distance)}, valeur: {Distance} - {type(Angle)}, valeur: {Angle}")
                    
                    print(abs(float(Distance)) , cap)
                    move_to_target(abs(float(Distance)),float(cap))

        except BrokenPipeError:
            print("\nLe pipe a été fermé par le récepteur. Arrèt du programme.", file=sys.stderr)
            sys.exit(1)  # Quitte proprement
        except KeyboardInterrupt:
            print("\nProgramme arrété par l'utilisateur.", file=sys.stderr)
            sys.exit(0)
            print(distance,cap)
                
