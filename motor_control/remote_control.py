import evdev #le module evdev est explique ici: https://www.youtube.com/watch?v=2F4M-7IGlrc
import numpy as np
import pygame
import time
import gpiod
from gpiod.line import Direction, Value
import threading
import subprocess
import re
import os

# On a Raspberry Pi 5, the 40-pin header's GPIO lines are owned by a
# separate chip (RP1, the "southbridge"), exposed as its own /dev/gpiochipN
# -- but WHICH number depends on the OS/kernel version, not the hardware:
# early Pi 5 images exposed it as gpiochip4, while a kernel/device-tree
# change (mid-2024 onward) moved it back to gpiochip0 for consistency with
# older Pi models, pushing unrelated internal chips to gpiochip10+. So a
# number that's correct today can silently stop being correct after a
# `apt upgrade` -- opening the wrong chip either fails outright (line
# offset out of range, or the device simply doesn't exist) or, worse,
# succeeds on a real but unrelated chip, silently doing nothing to the
# actual motor pins. The robust fix (Raspberry Pi's own recommendation)
# is to identify the chip by its driver *label*, "pinctrl-rp1", rather
# than a hardcoded number.
RP1_GPIOCHIP_LABEL = "pinctrl-rp1"
# Used only if auto-detection below fails (e.g. `gpiodetect` isn't
# installed, or this runs on a non-Pi5 board) -- matches what's been
# confirmed to work on this robot's Pi 5 as of 2026-09. Must be a full
# device path: gpiod v2's Chip() (unlike the older v1 API) does NOT
# resolve a bare name like "gpiochip0" against /dev/ on its own -- it
# passes the string straight to the OS open() call, so "gpiochip0"
# without the leading "/dev/" raises FileNotFoundError (confirmed by
# testing this for real on the robot's Pi 5, 2026-09-06).
FALLBACK_GPIOCHIP = "/dev/gpiochip0"


def detect_rp1_gpiochip(fallback=FALLBACK_GPIOCHIP):
    """Returns the full gpiochip device path (e.g. "/dev/gpiochip0")
    whose driver label is "pinctrl-rp1", by parsing `gpiodetect`'s output
    -- e.g. a line like "gpiochip0 [pinctrl-rp1] (54 lines)" becomes
    "/dev/gpiochip0". Can be overridden at any time with the
    ROBOT_GPIOCHIP environment variable (useful for testing off a real
    Pi 5, or if a future OS image renames the label) -- when set, it is
    used exactly as given (include the "/dev/" prefix yourself). Falls
    back to `fallback` if gpiodetect isn't available or nothing matches
    -- this never raises, so a detection glitch degrades to "try the
    last known-good chip" instead of crashing before the gamepad even
    starts."""
    override = os.environ.get("ROBOT_GPIOCHIP")
    if override:
        return override

    try:
        output = subprocess.run(
            ["gpiodetect"], capture_output=True, text=True, timeout=5, check=True
        ).stdout
    except (FileNotFoundError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return fallback

    for line in output.splitlines():
        match = re.match(r"(gpiochip\d+)\s+\[" + re.escape(RP1_GPIOCHIP_LABEL) + r"\]", line)
        if match:
            return f"/dev/{match.group(1)}"
    return fallback


class Remote():

    def __init__(self):
        self.chip = gpiod.Chip(detect_rp1_gpiochip())
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
                        self.M1_S1.set_value(self.MOTOR1_SENS1, Value.INACTIVE)
                    else:
                        self.M1_S1.set_value(self.MOTOR1_SENS1, Value.ACTIVE)
                elif self.dutyCycleLeft < -20:
                    if PWM_Counter > abs(self.dutyCycleLeft):
                        self.M1_S2.set_value(self.MOTOR1_SENS2, Value.INACTIVE)
                    else:
                        self.M1_S2.set_value(self.MOTOR1_SENS2, Value.ACTIVE)
                else:
                    self.M1_S1.set_value(self.MOTOR1_SENS1, Value.INACTIVE)
                    self.M1_S2.set_value(self.MOTOR1_SENS2, Value.INACTIVE)

                if self.dutyCycleRight > 20:
                    if PWM_Counter > self.dutyCycleRight:
                        self.M2_S1.set_value(self.MOTOR2_SENS1, Value.INACTIVE)
                    else:
                        self.M2_S1.set_value(self.MOTOR2_SENS1, Value.ACTIVE)
                elif self.dutyCycleRight < -20:
                    if PWM_Counter > abs(self.dutyCycleRight):
                        self.M2_S2.set_value(self.MOTOR2_SENS2, Value.INACTIVE)
                    else:
                        self.M2_S2.set_value(self.MOTOR2_SENS2, Value.ACTIVE)
                else:
                    self.M2_S1.set_value(self.MOTOR2_SENS1, Value.INACTIVE)
                    self.M2_S2.set_value(self.MOTOR2_SENS2, Value.INACTIVE)
    
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
                    # BUG FIX (2026-09-06): rewritten for libgpiod v2's
                    # Python API -- chip.get_line(offset).request(type=
                    # gpiod.LINE_REQ_DIR_OUT) is the OLD (v1) API and
                    # doesn't exist in v2 (confirmed installed on the
                    # robot's Pi 5: pygame/gpiod stack trace showed
                    # gpiod/chip.py, the v2 package layout). v2 requests
                    # lines via chip.request_lines(consumer=...,
                    # config={offset: gpiod.LineSettings(...)}), which
                    # returns a LineRequest object -- see set_value()
                    # calls below, also updated to v2's
                    # set_value(offset, Value.ACTIVE/INACTIVE) signature
                    # instead of v1's set_value(0/1).
                    self.M1_S1 = self.chip.request_lines(
                        consumer="M1_S1",
                        config={self.MOTOR1_SENS1: gpiod.LineSettings(direction=Direction.OUTPUT)},
                    )
                    self.M1_S2 = self.chip.request_lines(
                        consumer="M1_S2",
                        config={self.MOTOR1_SENS2: gpiod.LineSettings(direction=Direction.OUTPUT)},
                    )
                    self.M2_S1 = self.chip.request_lines(
                        consumer="M2_S1",
                        config={self.MOTOR2_SENS1: gpiod.LineSettings(direction=Direction.OUTPUT)},
                    )
                    self.M2_S2 = self.chip.request_lines(
                        consumer="M2_S2",
                        config={self.MOTOR2_SENS2: gpiod.LineSettings(direction=Direction.OUTPUT)},
                    )

                    # BUG FIX (2026-09-06): pwm(self) takes no arguments
                    # beyond self -- it reads self.dutyCycleLeft/Right
                    # live on every loop iteration instead. Passing
                    # args=(self.dutyCycleLeft, self.dutyCycleRight) here
                    # made the thread crash immediately with a TypeError
                    # the moment it started (2 extra positional args to a
                    # 0-argument method), so the PWM loop never actually
                    # ran a single cycle -- silently, since a background
                    # thread's exception only prints a traceback to
                    # stderr instead of stopping the program, easy to
                    # miss among this loop's own prints. This was true
                    # regardless of which gpiochip was used.
                    pwm_tread = threading.Thread(target=self.pwm)
                    pwm_tread.start()

            try:
                while True:
                    try:
                        # Reading the remote status
                        count = pygame.joystick.get_count()
                    except KeyboardInterrupt:
                        self.M1_S1.set_value(self.MOTOR1_SENS1, Value.INACTIVE)
                        self.M2_S1.set_value(self.MOTOR2_SENS1, Value.INACTIVE)
                        self.M1_S2.set_value(self.MOTOR1_SENS2, Value.INACTIVE)
                        self.M2_S2.set_value(self.MOTOR2_SENS2, Value.INACTIVE)
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
                                    self.M1_S1.set_value(self.MOTOR1_SENS1, Value.INACTIVE)
                                    self.M2_S1.set_value(self.MOTOR2_SENS1, Value.INACTIVE)
                                    self.M1_S2.set_value(self.MOTOR1_SENS2, Value.INACTIVE)
                                    self.M2_S2.set_value(self.MOTOR2_SENS2, Value.INACTIVE)
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
                                    self.M1_S1.set_value(self.MOTOR1_SENS1, Value.INACTIVE)
                                    self.M2_S1.set_value(self.MOTOR2_SENS1, Value.INACTIVE)
                                    self.M1_S2.set_value(self.MOTOR1_SENS2, Value.INACTIVE)
                                    self.M2_S2.set_value(self.MOTOR2_SENS2, Value.INACTIVE)
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
                self.M1_S1.set_value(self.MOTOR1_SENS1, Value.INACTIVE)
                self.M2_S1.set_value(self.MOTOR2_SENS1, Value.INACTIVE)
                self.M1_S2.set_value(self.MOTOR1_SENS2, Value.INACTIVE)
                self.M2_S2.set_value(self.MOTOR2_SENS2, Value.INACTIVE)
                try:
                    pygame.quit()
                except:
                    print("No device connected")
                finally:
                    print("Good Bye")

 
