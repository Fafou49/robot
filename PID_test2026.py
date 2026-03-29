import sys
import keyboard
import time
import math

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

def move_to_target( current_distance, current_angle):
    dt = 1/5  # Pas de temps (en secondes)
    
    while True:
        # Mise à jour des PID
        
        speed = pid_distance.update(current_distance, dt)
        angular_velocity = pid_angle.update(current_angle, dt)

        # Simulation : mise à jour de la position (à remplacer par ton système rèel)
        current_distance += speed * dt
        current_angle += angular_velocity * dt

        # Affichage (pour le débogage)
        print(f"Distance: {current_distance:.2f}, Angle: {current_angle:.2f}")

        # Condition d'arrêt
        if abs(pid_distance.setpoint - current_distance) < 0.01 and abs(pid_angle.setpoint - current_angle) < 0.01:
            print("Cible atteinte !")
            break

        time.sleep(dt)


time.sleep(3)
pid_distance = PIDController(kp=1.0, ki=0.1, kd=0.01, setpoint=0)
pid_angle = PIDController(kp=0.5, ki=0.01, kd=0.1, setpoint=0)

try:
    while True:
        if keyboard.is_pressed("esc"):
            print("\nBoucle arrètée par l'utilisateur.")
            break
        for line in sys.stdin:
            #print(line.strip().split(','))
            Distance, Angle =line.strip().split(',')
            print(f"{type(Distance)}, valeur: {Distance} - {type(Angle)}, valeur: {Angle}")
            
            #print(Distance , Angle)
            move_to_target(abs(float(Distance)),float(Angle))

except BrokenPipeError:
    print("\nLe pipe a été fermé par le récepteur. Arrèt du programme.", file=sys.stderr)
    sys.exit(1)  # Quitte proprement
except KeyboardInterrupt:
    print("\nProgramme arrété par l'utilisateur.", file=sys.stderr)
    sys.exit(0)
