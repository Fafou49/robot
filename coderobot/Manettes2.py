# Controller
import evdev #le module evdev est expliqué ici: https://www.youtube.com/watch?v=2F4M-7IGlrc
import numpy as np
import pygame

device = evdev.InputDevice('/dev/input/event5')
print(device, " connected")
pygame.init()
pygame.joystick.init()
WIDTH, HEIGHT = 1000, 680
FPS = 60
state_joystick = False
count = pygame.joystick.get_count()

# Vérifier si une manette est connectée
if count!=0:
    # Utiliser la première manette trouvée
    state_joystick = True
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

speed_x = 120
speed_y = 120
x1, y1 = WIDTH // 2, HEIGHT // 2
x2, y2 = WIDTH // 2, HEIGHT // 2
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Manettes")

sprite1 = pygame.Rect(x1, y1, 50, 50)
sprite2 = pygame.Rect(x2, y2, 50, 50)
timer = pygame.time.Clock()

running = True

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            pygame.quit()
    if state_joystick:
        joystickGauche_x = joystick.get_axis(0)
        joystickGauche_y = joystick.get_axis(1)
        joystickDroit_x = joystick.get_axis(2)
        joystickDroit_y = joystick.get_axis(3)
        TriggerL = joystick.get_axis(4)
        TriggerR = joystick.get_axis(5)

        bouttonA = joystick.get_button(0)
        bouttonX = joystick.get_button(1)
        bouttonB = joystick.get_button(2)
        bouttonY = joystick.get_button(3)
        bouttonLT = joystick.get_button(4)
        bouttonRT = joystick.get_button(5)
        bouttonSelect = joystick.get_button(6)
        bouttonStart = joystick.get_button(7)
        bouttonLeftBumper = joystick.get_button(8)
        bouttonRightBumper = joystick.get_button(9)

        if joystickDroit_x >= 0.2:
            x1 += (speed_x / FPS)
        elif joystickDroit_x <= -0.2:
            x1 -= (speed_x / FPS)

        if joystickDroit_y >= 0.2:
            y1 += (speed_y / FPS)
        elif joystickDroit_y <= -0.2:
            y1 -= (speed_y / FPS)

        if joystickGauche_x >= 0.2:
            x2 += (speed_x / FPS)
        elif joystickGauche_x <= -0.2:
            x2 -= (speed_x / FPS)

        if joystickGauche_y >= 0.2:
            y2 += (speed_y / FPS)
        elif joystickGauche_y <= -0.2:
            y2 -= (speed_y / FPS)

        if x1 > WIDTH - 25:
            x1 = WIDTH - 25
        elif x1 < 0:
            x1 = 0
        if y1 > HEIGHT - 25:
            y1 = HEIGHT - 25
        elif y1 < 0:
            y1 = 0

        if x2 > WIDTH - 25:
            x2 = WIDTH - 25
        elif x2 < 0:
            x2 = 0
        if y2 > HEIGHT - 25:
            y2 = HEIGHT - 25
        elif y2 < 0:
            y2 = 0

        sprite1.x = x1
        sprite1.y = y1
        sprite2.x = x2
        sprite2.y = y2

        screen.fill((255, 255, 255))
        pygame.draw.rect(screen, pygame.Color("black"), sprite1)
        pygame.draw.rect(screen, pygame.Color("black"), sprite2)
        if bouttonA:
            pygame.draw.rect(screen, pygame.Color("green"), sprite1)
        if bouttonX:
            pygame.draw.rect(screen, pygame.Color("red"), sprite1)
        if bouttonB:
            pygame.draw.rect(screen, pygame.Color("blue"), sprite1)
        if bouttonY:
            pygame.draw.rect(screen, pygame.Color("yellow"), sprite1)
        if bouttonLT:
            pygame.draw.rect(screen, pygame.Color("orange"), sprite1)
        if bouttonRT:
            pygame.draw.rect(screen, pygame.Color("brown"), sprite1)
        if bouttonSelect:
            pygame.draw.rect(screen, pygame.Color("grey"), sprite1)
        if bouttonStart:
            pygame.draw.rect(screen, pygame.Color("pink"), sprite1)
        if bouttonLeftBumper:
            pygame.draw.rect(screen, pygame.Color(255, 0, 128), sprite2)
        if bouttonRightBumper:
            pygame.draw.rect(screen, pygame.Color(100, 100, 100), sprite2)

        print(joystickGauche_y, joystickDroit_y)

        pygame.display.update()
        timer.tick(FPS)
