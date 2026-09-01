import sys
import time
import math



class PIDController:
    def __init__(self, setpoint : float , kp, ki, kd,):
        self.kp = kp  # Coefficient proportionnel
        self.ki = ki  # Coefficient intégral
        self.kd = kd  # Coefficient dérivé
        self.setpoint = setpoint  # Valeur cible (distance ou angle)
        self.integral = 0
        self.previous_error = None  # None = pas encore de mesure precedente (voir update())
        self.last_time = time.time()  # Stocke le temps ici
        
    def set_setpoint(self, setpoint : float):
        self.setpoint=setpoint
        
    def update(self, measured_value):
        
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0:
            dt = 0.01  # Valeur par défaut pour éviter la division par zéro
    
        error = self.setpoint - measured_value
        if self.previous_error is None:
            # Premier appel : pas de mesure precedente valable, donc pas de
            # terme derive a calculer (sinon "derivative kick" enorme si
            # l'erreur de depart est deja grande, ex: 5 m ou 45 degres).
            self.previous_error = error
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        
        return output
        


def move_to_target( current_distance : float, current_angle : float):


    # Mise à jour des PID
        
    speed = pid_distance.update(current_distance)
    angular_velocity = pid_angle.update(current_angle)
    # Limiter les valeurs de sortie si nécessaire
    speed = max(min(speed, 2.0), -2.0) # m/s
    angular_velocity = max(min(angular_velocity, 360), -360)# m/s

    #current_distance = pid_distance.setpoint
    #current_angle = pid_angle.setpoint
    #current_angle = (current_angle + 180) % 360 - 180  # [-180, 180]      
        
        
    # Conversion pour les moteurs (exemple pour roues différentielles)
    wheel_base = 0.59  # Distance entre les roues (en mètres)
        
    # Convertir speed (m/s) en PWM (0-255)
    max_speed = 2.0  # Vitesse max en m/s
    speed_pwm = (speed / max_speed) * 255  # échelle de 0 à 255

    # Convertir angular_velocity (°/s) en PWM
    max_angular_velocity = 360  # Vitesse angulaire max en °/s
    angular_pwm = (angular_velocity / max_angular_velocity) * 255

    # Appliquer aux moteurs
    left_speed = speed_pwm + angular_pwm
    right_speed = speed_pwm - angular_pwm

    # Rebornage : speed_pwm et angular_pwm peuvent chacun atteindre +/-255,
    # donc leur somme/difference peut depasser +/-255 (jusqu'a +/-510) alors
    # que motor_control/pwm.py attend une valeur physique dans +/-255.
    left_speed = max(min(left_speed, 255), -255)
    right_speed = max(min(right_speed, 255), -255)


    print(f"{time.time()},{left_speed:.2f},{right_speed:.2f},{current_distance:.2f},{current_angle:.2f},{speed:.2f},{angular_velocity:.2f} ",flush=True)
    sys.stdout.flush() 



# Initialisation des cibles par défaut
distance_target = 0.0
angle_target = 0.0

# Initialisation des PID avec des consignes par défaut
# Valeurs kp/ki/kd estimees par simulation (voir pid/simulate_motor_commands.py
# et le README, section "Reglage des PID hors robot") : pas de depassement ni
# de saturation moteur avec ces coefficients, sur le modele generique simule.
# A valider ensuite sur le robot reel, a basse vitesse, avant usage complet.
pid_distance = PIDController(kp=1, ki=0, kd=0.5, setpoint=distance_target)
pid_angle = PIDController(kp=0.5, ki=0, kd=1, setpoint=angle_target)
#------------------------------MAIN-------------------------------        
def main():


    # Lire la première ligne pour initialiser les cibles
    try:
        
        first_line = sys.stdin.readline().strip() 
        seconde_line = sys.stdin.readline().strip()       
        if seconde_line:
            parts = seconde_line.split(',')
            if len(parts) >= 2:
                distance_target = float(parts[0].strip())
                angle_target = float(parts[1].strip())
                print(distance_target,",",angle_target)
                # Mettre à jour les consignes des PID
                pid_distance.set_setpoint(distance_target)                
                pid_angle.set_setpoint(angle_target)
            else:
                print(f"[ERREUR] Ligne d'initialisation invalide : {seconde_line!r}", file=sys.stderr)
    except Exception as e:
        print(f"[ERREUR] Lecture de l'initialisation : {e}", file=sys.stderr)
        sys.exit(1)

    # Boucle principale pour traiter les entrées en temps réel
    try:
        while True:
            line = sys.stdin.readline().strip()
              
            if not line:
                continue  # Ignorer les lignes vides

            parts = line.split(',')
            if len(parts) < 2:
                print(f"[IGNORE] Ligne invalide : {line!r}", file=sys.stderr)
                continue

            try:
                current_distance = float(parts[0].strip())
                current_angle = float(parts[1].strip())
            except ValueError as e:
                print(f"[ERREUR] Conversion impossible : {line!r} -> {e}", file=sys.stderr)
                continue

            # Appeler move_to_target avec les valeurs actuelles
            move_to_target(current_distance,current_angle)

    except BrokenPipeError:
        print("\n[ERREUR] Le pipe a été fermé. Arrèt du programme.", file=sys.stderr)
        sys.exit(0)
    except KeyboardInterrupt:
        print("\n[INFO] Arrèt demandé par l'utilisateur (CTRL+C).", file=sys.stderr)
        sys.exit(0)
    except Exception as e:
        print(f"\n[ERREUR] Erreur inattendue : {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == '__main__':
    main()
