import gpiod
import time


MOTOR1_SENS1 = 5 # /!\ bien prendre les n° de broches GPIO et non hardware
MOTOR1_SENS2 = 6 # /!\ bien prendre les n° de broches GPIO et non hardware
MOTOR2_SENS1 = 19 # /!\ bien prendre les n° de broches GPIO et non hardware
MOTOR2_SENS2 = 26 # /!\ bien prendre les n° de broches GPIO et non hardware

chip = gpiod.Chip('gpiochip4')

M1_S1 = chip.get_line(MOTOR1_SENS1)
M1_S1.request(consumer="M1_S1", type=gpiod.LINE_REQ_DIR_OUT)

M1_S2 = chip.get_line(MOTOR1_SENS2)
M1_S2.request(consumer="M1_S2", type=gpiod.LINE_REQ_DIR_OUT)

M2_S1 = chip.get_line(MOTOR2_SENS1)
M2_S1.request(consumer="M2_S1", type=gpiod.LINE_REQ_DIR_OUT)

M2_S2 = chip.get_line(MOTOR2_SENS2)
M2_S2.request(consumer="M2_S2", type=gpiod.LINE_REQ_DIR_OUT)

try:
	
	time.sleep(5)
	M1_S1.set_value(1)
	print(time.strftime("%H:%M:%S"), "M1_S1")
	time.sleep(5)
	M1_S1.set_value(0)
	M1_S2.set_value(1)
	print(time.strftime("%H:%M:%S"),"M1_S2")
	time.sleep(5)
	M1_S2.set_value(0)
	print(time.strftime("%H:%M:%S"),"neutre")
	time.sleep(5)
	
	M2_S1.set_value(1)
	print(time.strftime("%H:%M:%S"), "M2_S1")
	time.sleep(5)
	M2_S1.set_value(0)
	M2_S2.set_value(1)
	print(time.strftime("%H:%M:%S"),"M2_S2")
	time.sleep(5)
	M2_S2.set_value(0)
	print(time.strftime("%H:%M:%S"),"neutre")
	time.sleep(5)
	
	M1_S1.set_value(1)
	print(time.strftime("%H:%M:%S"), "M1_S1")
	M2_S1.set_value(1)
	print(time.strftime("%H:%M:%S"), "M2_S1")
	time.sleep(5)
	
	M1_S1.set_value(0)
	print(time.strftime("%H:%M:%S"),"M1 neutre") 
	M2_S1.set_value(0)
	print(time.strftime("%H:%M:%S"),"M2 neutre")
	
	time.sleep(5)
	
	M1_S2.set_value(1)
	print(time.strftime("%H:%M:%S"), "M1_S2")
	M2_S2.set_value(1)
	print(time.strftime("%H:%M:%S"), "M2_S2")
	time.sleep(5)
	
	M1_S2.set_value(0)
	print(time.strftime("%H:%M:%S"),"M1 neutre") 
	M2_S2.set_value(0)
	print(time.strftime("%H:%M:%S"),"M2 neutre")
		
finally:
	M1_S1.release()
