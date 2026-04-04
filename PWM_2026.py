import gpiod
import sys
import time
import keyboard
import threading
from gpiod.line import Direction, Value

def pwm(dutyCycleLeft,dutyCycleRight):
        while True:  
            
                if type(dutyCycleLeft) is not float:
                    dutyCycleLeft=float(dutyCycleLeft)
                if type(dutyCycleRight) is not float:
                    dutyCycleRight=float(dutyCycleRight)
                # 5.1 - Counting from 0 to 254 all the time  
                for PWM_Counter in range(254):
                        print ("dutyCycleRight : " , dutyCycleRight," / " ,"dutyCycleLeft : ",dutyCycleLeft )
                        if dutyCycleLeft>20:
                                if PWM_Counter>dutyCycleLeft:
                                        M1_S1.set_value(MOTOR1_SENS1, Value.INACTIVE)
                                else:
                                        M1_S1.set_value(MOTOR1_SENS1, Value.ACTIVE)
                        elif dutyCycleLeft<-20:
                                if PWM_Counter>abs(dutyCycleLeft):
                                        M1_S2.set_value(MOTOR1_SENS2, Value.INACTIVE) 
                                else:
                                        M1_S2.set_value(MOTOR1_SENS2, Value.ACTIVE) 
                        else:
                                M1_S1.set_value(MOTOR1_SENS1, Value.INACTIVE) 
                                M1_S2.set_value(MOTOR1_SENS2, Value.INACTIVE) 
                        
                        if dutyCycleRight>20:
                                if PWM_Counter>dutyCycleRight:
                                        M2_S1.set_value(MOTOR2_SENS1, Value.INACTIVE)
                                else:
                                        M2_S1.set_value(MOTOR2_SENS1, Value.ACTIVE)
                        elif dutyCycleRight<-20:
                                if PWM_Counter>abs(dutyCycleRight):
                                        M2_S2.set_value(MOTOR2_SENS2, Value.INACTIVE)
                                else:
                                        M2_S2.set_value(MOTOR2_SENS2, Value.ACTIVE)
                        else:
                                M2_S1.set_value(MOTOR2_SENS1, Value.INACTIVE)
                                M2_S2.set_value(MOTOR2_SENS2, Value.INACTIVE)
                        time.sleep(0.001)

verrou = threading.Lock()
dutyCycleLeft=0
dutyCycleRight=0

MOTOR1_SENS1 = 14
MOTOR1_SENS2 = 15
MOTOR2_SENS1 = 2
MOTOR2_SENS2 = 3

chip = gpiod.Chip('/dev/gpiochip4')
    
M1_S1=chip.request_lines(consumer="MOTOR1_SENS1", config={
        MOTOR1_SENS1: gpiod.LineSettings(direction=Direction.OUTPUT)})

M1_S2=chip.request_lines(consumer="MOTOR1_SENS2", config={
        MOTOR1_SENS2: gpiod.LineSettings(direction=Direction.OUTPUT)})

M2_S1=chip.request_lines(consumer="MOTOR2_SENS1", config={
        MOTOR2_SENS1: gpiod.LineSettings(direction=Direction.OUTPUT)})
        
M2_S2=chip.request_lines(consumer="MOTOR2_SENS2", config={
        MOTOR2_SENS2: gpiod.LineSettings(direction=Direction.OUTPUT)})
        
        


try:
    

    while True:

        
        for DuttyCycles in sys.stdin:
            
            with verrou:
                dutyCycleLeft, dutyCycleRight =  DuttyCycles.strip().split(',')[0],DuttyCycles.strip().split(',')[0]
                pwm_tread = threading.Thread(target=pwm, args=(dutyCycleLeft, dutyCycleRight,))
                pwm_tread.start()
        time.sleep(1/5)        
        
    
except BrokenPipeError:
    verrou = True
    threading.join()
    M1_S1.set_value(MOTOR1_SENS1, Value.INACTIVE)
    M2_S1.set_value(MOTOR2_SENS1, Value.INACTIVE)
    M1_S2.set_value(MOTOR1_SENS2, Value.INACTIVE)
    M2_S2.set_value(MOTOR2_SENS2, Value.INACTIVE)
    chip.close()

    print("\nLe pipe a été fermé par le récepteur. Arrèt du programme.", file=sys.stderr)
    sys.exit(1)  # Quitte proprement
except KeyboardInterrupt:
    verrou = True
    threading.join()
    M1_S1.set_value(MOTOR1_SENS1, Value.INACTIVE)
    M2_S1.set_value(MOTOR2_SENS1, Value.INACTIVE)
    M1_S2.set_value(MOTOR1_SENS2, Value.INACTIVE)
    M2_S2.set_value(MOTOR2_SENS2, Value.INACTIVE)
    chip.close()
    print("\nProgramme arrété par l'utilisateur.", file=sys.stderr)
    sys.exit(0)
finally:
    verrou = True
    threading.join()
    M1_S1.set_value(MOTOR1_SENS1, Value.INACTIVE)
    M2_S1.set_value(MOTOR2_SENS1, Value.INACTIVE)
    M1_S2.set_value(MOTOR1_SENS2, Value.INACTIVE)
    M2_S2.set_value(MOTOR2_SENS2, Value.INACTIVE)
    chip.close()
    

 
 
