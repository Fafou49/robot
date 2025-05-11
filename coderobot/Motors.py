import time
import gpiod
import spidev 
import datetime

def init_pot():
    TCON = 0b0100 #add=4
    Status_REG=0b0101 #add=5
    
    #TCON
    
    #CS_bar.set_value(0)
    
    print("TCON (WRITE):")
    _dev.writebytes2([0b010011,0b11111111])
    _dev.writebytes2([0b010000])
    print ("TCON= ",_dev.readbytes(8))
    #Status_REG
    print("SREG (WRITE):")
    _dev.writebytes2([0b010111,0b11111111])
    print("SREG (READ):")
    _dev.writebytes2([0b010100])
    print ("SREG= ",_dev.readbytes(8))
    time.sleep(5)


def test_increment():
    
    print("test_increment:")
    response=_dev.xfer2([0b000001])
    print("1 :",response)
    response=_dev.xfer2([0b000101])
    print("2 :",response)   
    time.sleep(0.1)

def write_pot(value):
    msb = (value >> 8) & 0xFF
    lsb = value & 0xFF
    print(_dev.xfer2([msb, lsb]))
    

	
MOTOR1_SENS1 = 5 # /!\ bien prendre les n° de broches GPIO et non hardware
MOTOR1_SENS2 = 6 #
MOTOR2_SENS1 = 26 # 
MOTOR2_SENS2 = 19 # 
cs_bar = 8
MOSI_pin=10
MISO_pin=9
CLK_pin=11


chip = gpiod.Chip('gpiochip4')
_dev = spidev.SpiDev(0,0)     # Start a SpiDev device
_dev.open(0,0)
_dev.mode = 0b00               # Set SPI mode (phase)

'''Remarks: The 125 MHz default value is not sustainable on a
Raspberry Pi and it must be changed to a reasonable value. The
maximum speed appears to be about 32 MHz on the Raspberry Pi'''

_dev.max_speed_hz = 3800 # Set the data rate
T =1/_dev.max_speed_hz
_dev.bits_per_word = 8    # Number of bit per word. ALWAYS 8


M1_S1 = chip.get_line(MOTOR1_SENS1)
M1_S1.request(consumer="M1_S1", type=gpiod.LINE_REQ_DIR_OUT)

M1_S2 = chip.get_line(MOTOR1_SENS2)
M1_S2.request(consumer="M1_S2", type=gpiod.LINE_REQ_DIR_OUT)

M2_S1 = chip.get_line(MOTOR2_SENS1)
M2_S1.request(consumer="M2_S1", type=gpiod.LINE_REQ_DIR_OUT)

M2_S2 = chip.get_line(MOTOR2_SENS2)
M2_S2.request(consumer="M2_S2", type=gpiod.LINE_REQ_DIR_OUT)

'''MOSI = chip.get_line(MOSI_pin)
MOSI.request(consumer="MOSI", type=gpiod.LINE_REQ_DIR_OUT)

MISO = chip.get_line(MISO_pin)
MISO.request(consumer="MISO", type=gpiod.LINE_REQ_DIR_IN)

SCLK = chip.get_line(CLK_pin)
SCLK.request(consumer="SCLK", type=gpiod.LINE_REQ_DIR_OUT)
'''
#CS_bar = chip.get_line(cs_bar)
#CS_bar.request(consumer="CS_bar", type=gpiod.LINE_REQ_DIR_OUT)

try:
    init_pot()
    M1_S1.set_value(1)
    M2_S1.set_value(1)
    
    i=0
    for i in range(255):
        test_increment()
        print(i)
        
    M1_S1.set_value(0)
    M2_S1.set_value(0)
    

    _dev.close()
    time.sleep(3)  
        
except KeyboardInterrupt:
    M1_S1.set_value(0)
    M2_S1.set_value(0)
    time.sleep(10)
    #_dev.close()
    #sys.exit(0)
