import evdev #le module evdev est explique ici: https://www.youtube.com/watch?v=2F4M-7IGlrc
import numpy as np
import pygame
import time
import gpiod
import threading

class Remote():

    def __init__(self):
        self.chip = gpiod.Chip('gpiochip4')
        self.MOTOR1_SENS1 = 14
        self.MOTOR1_SENS2 = 15
        self.MOTOR2_SENS1 = 2
        self.MOTOR2_SENS2 = 3
        self.dutyCycleLeft = 0
        self.dutyCycleRight = 0
        self.device = 0
        self.verrou = threading.Lock()
        
        
    def pwm(self):
        while True:
            # 5.1 - Counting from 0 to 255 all the time
            for PWM_Counter in range(255):
                print("dutyCycleRight : ", self.dutyCycleRight, " / ", "dutyCycleLeft : ", self.dutyCycleLeft)
                if self.dutyCycleLeft > 20:
                    if PWM_Counter > self.dutyCycleLeft:
                        self.M1_S1.set_value(0)
                    else:
                        self.M1_S1.set_value(1)
                elif self.dutyCycleLeft < -20:
                    if PWM_Counter > abs(self.dutyCycleLeft):
                        self.M1_S2.set_value(0)
                    else:
                        self.M1_S2.set_value(1)
                else:
                    self.M1_S1.set_value(0)
                    self.M1_S2.set_value(0)

                if self.dutyCycleRight > 20:
                    if PWM_Counter > self.dutyCycleRight:
                        self.M2_S1.set_value(0)
                    else:
                        self.M2_S1.set_value(1)
                elif self.dutyCycleRight < -20:
                    if PWM_Counter > abs(self.dutyCycleRight):
                        self.M2_S2.set_value(0)
                    else:
                        self.M2_S2.set_value(1)
                else:
                    self.M2_S1.set_value(0)
                    self.M2_S2.set_value(0)
    
    def fonction1(self):
            # 1 - testing the remote connection
        while self.device == 0:
            try:
                self.device = evdev.InputDevice('/dev/input/event5')
            except:
                print("No device connected")
            else:
                print(self.device, " connected")
                pygame.init()
                pygame.joystick.init()
                FPS = 5
                state_joystick = False
            finally:
                if self.device != 0:
                    print("starting")
                else:
                    print("new try")

                # 2 - IF the remote is connected, THEN
                if self.device != 0:
                    # setting the OUTPUTS
                    self.M1_S1 = self.chip.get_line(self.MOTOR1_SENS1)
                    self.M1_S1.request(consumer="M1_S1", type=gpiod.LINE_REQ_DIR_OUT)

                    self.M1_S2 = self.chip.get_line(self.MOTOR1_SENS2)
                    self.M1_S2.request(consumer="M1_S2", type=gpiod.LINE_REQ_DIR_OUT)

                    self.M2_S1 = self.chip.get_line(self.MOTOR2_SENS1)
                    self.M2_S1.request(consumer="M2_S1", type=gpiod.LINE_REQ_DIR_OUT)

                    self.M2_S2 = self.chip.get_line(self.MOTOR2_SENS2)
                    self.M2_S2.request(consumer="M2_S2", type=gpiod.LINE_REQ_DIR_OUT)

                    pwm_tread = threading.Thread(target=self.pwm, args=(self.dutyCycleLeft, self.dutyCycleRight,))
                    pwm_tread.start()

            try:
                while True:
                    try:
                        # Reading the remote status
                        count = pygame.joystick.get_count()
                    except KeyboardInterrupt:
                        self.M1_S1.set_value(0)
                        self.M2_S1.set_value(0)
                        self.M1_S2.set_value(0)
                        self.M2_S2.set_value(0)
                        pygame.quit()
                    except:
                        print("device disconected")
                        break
                    else:
                        if count != 0:
                            # using the remote found
                            state_joystick = True
                            joystick = pygame.joystick.Joystick(0)
                            joystick.init()
                        timer = pygame.time.Clock()

                        running = True
                        # 4.1 - reading all the time the remote signals
                        while running:
                            for event in pygame.event.get():
                                if event.type == pygame.QUIT:
                                    running = False
                                    self.M1_S1.set_value(0)
                                    self.M2_S1.set_value(0)
                                    self.M1_S2.set_value(0)
                                    self.M2_S2.set_value(0)
                                    pygame.quit()
                            if state_joystick:
                                # 3 - pushing the remote signals to understandable variables
                                joystickGauche_x = joystick.get_axis(0)
                                joystickGauche_y = joystick.get_axis(1)
                                joystickDroit_x = joystick.get_axis(2)
                                joystickDroit_y = joystick.get_axis(3)
                                TriggerL = joystick.get_axis(4)
                                TriggerR = joystick.get_axis(5)

                                bouttonA = joystick.get_button(0)
                                # print("bouttonA 0 :",bouttonA)
                                bouttonB = joystick.get_button(1)
                                # print("bouttonB 1 :",bouttonB)
                                bouttonX = joystick.get_button(3)
                                # print("bouttonX 3 :",bouttonX)
                                bouttonY = joystick.get_button(4)
                                # print("bouttonY 4 :",bouttonY)
                                bouttonLT = joystick.get_button(6)
                                # print("bouttonLT 6 :",bouttonLT)
                                bouttonRT = joystick.get_button(7)
                                # print("bouttonRT 7 :",bouttonRT)
                                bouttonSelect = joystick.get_button(10)
                                # print("bouttonSelect 10 :",bouttonSelect)
                                bouttonStart = joystick.get_button(11)
                                # print("bouttonStart 11 :",bouttonStart)
                                bouttonLeftBumper = joystick.get_button(13)
                                # print("bouttonLeftBumper 13 :",bouttonLeftBumper)
                                bouttonRightBumper = joystick.get_button(14)
                                # print("bouttonRightBumper 14 :",bouttonRightBumper)

                                # finish the loop if signal "START"
                                if bouttonStart == 1:
                                    self.M1_S1.set_value(0)
                                    self.M2_S1.set_value(0)
                                    self.M1_S2.set_value(0)
                                    self.M2_S2.set_value(0)
                                    running = False
                                    pygame.quit()
                                    break

                                with self.verrou:
                                    # 4.2 Converting the Joysticks signals to the Motor values (-255 to 255)
                                    self.dutyCycleRight = int((-joystickDroit_y) * 255)
                                    self.dutyCycleLeft = int((-joystickGauche_y) * 255)

                                # wait the FPS time
                                timer.tick(FPS)

            # closing the process and turnning of the MOTORS
            finally:
                print("end!!!")
                self.M1_S1.set_value(0)
                self.M2_S1.set_value(0)
                self.M1_S2.set_value(0)
                self.M2_S2.set_value(0)
                try:
                    pygame.quit()
                except:
                    print("No device connected")
                finally:
                    print("Good Bye")

 
