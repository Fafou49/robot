import time
import gpiod


	
MOTOR1_SENS1 = 5 # /!\ bien prendre les n° de broches GPIO et non hardware
MOTOR1_SENS2 = 6 # /!\ bien prendre les n° de broches GPIO et non hardware
MOTOR2_SENS1 = 19 # /!\ bien prendre les n° de broches GPIO et non hardware
MOTOR2_SENS2 = 26 # /!\ bien prendre les n° de broches GPIO et non hardware
cs_bar = 7
MOSI_pin=10
MISO_pin=9
CLK_pin=11

#cs_bar = 18
#MOSI_pin=20
#MISO_pin=19
#CLK_pin=21


chip = gpiod.Chip('gpiochip4')


M1_S1 = chip.get_line(MOTOR1_SENS1)
M1_S1.request(consumer="M1_S1", type=gpiod.LINE_REQ_DIR_OUT)

M1_S2 = chip.get_line(MOTOR1_SENS2)
M1_S2.request(consumer="M1_S2", type=gpiod.LINE_REQ_DIR_OUT)

M2_S1 = chip.get_line(MOTOR2_SENS1)
M2_S1.request(consumer="M2_S1", type=gpiod.LINE_REQ_DIR_OUT)

M2_S2 = chip.get_line(MOTOR2_SENS2)
M2_S2.request(consumer="M2_S2", type=gpiod.LINE_REQ_DIR_OUT)

MOSI = chip.get_line(MOSI_pin)
MOSI.request(consumer="MOSI", type=gpiod.LINE_REQ_DIR_OUT)

MISO = chip.get_line(MISO_pin)
MISO.request(consumer="MISO", type=gpiod.LINE_REQ_DIR_OUT)

CLK = chip.get_line(CLK_pin)
CLK.request(consumer="CLK", type=gpiod.LINE_REQ_DIR_OUT)



try:
    M1_S2.set_value(1)
    M2_S2.set_value(1)

    time.sleep(2) 
    
    M1_S2.set_value(0)
    M2_S2.set_value(0)
    
    time.sleep(0.2) 
    
    M1_S1.set_value(1)
    M2_S1.set_value(1)

    time.sleep(2) 
    
    M1_S1.set_value(0)
    M2_S1.set_value(0)

     
        
except KeyboardInterrupt:
    M1_S2.set_value(0)
    M2_S2.set_value(0)
    time.sleep(10)
    #_dev.close()
    #sys.exit(0)
