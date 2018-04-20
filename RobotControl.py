import threading
import time
from inputs import get_gamepad
import RPi.GPIO as GPIO
import math

# To Update:
# Mac Terminal: cd ~/PycharmProjects/ME205_RC
#               git add -u
#               git commit -m "______"
#               git push -u origin master
# To Pull:
# Rpi Terminal: cd /home/pi/ME205_RobotControl
#               git pull
# To See Status: git status
# To Add a new file: git add {path to file}

gamepad_lock = threading.Lock()
gamepad_Lx = 0.0
gamepad_Ly = 0.0
gamepad_Rx = 0.0
gamepad_Ry = 0.0
gamepad_B = False
gamepad_back = False
gamepad_run = True

on = 0
off = 1
CW = 1
CCW = 0
direc = 25
enable = 23
step = 24
lCimPin = 12
rCimPin = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(enable, GPIO.OUT)
GPIO.setup(step, GPIO.OUT)
GPIO.setup(direc, GPIO.OUT)
GPIO.setup(lCimPin, GPIO.OUT)
GPIO.setup(rCimPin, GPIO.OUT)
Lmotor = GPIO.PWM(lCimPin, 80)
Rmotor = GPIO.PWM(rCimPin, 80)


def gamepad_loop():
    global gamepad_Lx
    global gamepad_Ly
    global gamepad_Rx
    global gamepad_Ry
    global gamepad_B
    global gamepad_back

    while gamepad_run:
        events = get_gamepad()
        for event in events:
            if str(event.code) == "ABS_X":
                gamepad_lock.acquire()
                gamepad_Lx = (event.state - (255.0 / 2.0))
                gamepad_lock.release()
            if str(event.code) == "ABS_Y":
                gamepad_lock.acquire()
                gamepad_Ly = -1 * (event.state - (255.0 / 2.0))
                gamepad_lock.release()
            if str(event.code) == "ABS_Z":
                gamepad_lock.acquire()
                gamepad_Rx = (event.state - (255.0 / 2.0))
                gamepad_lock.release()
            if str(event.code) == "ABS_RZ":
                gamepad_lock.acquire()
                gamepad_Ry = -1 * (event.state - (255.0 / 2.0))
                gamepad_lock.release()
            if str(event.code) == "BTN_C":
                gamepad_lock.acquire()
                gamepad_B = event.state
                gamepad_lock.release()
            if str(event.code) == "BTN_TL2":
                gamepad_lock.acquire()
                gamepad_back = event.state
                gamepad_lock.release()


def get_gamepad_input():
    gamepad_lock.acquire()
    lx, ly = gamepad_Lx, gamepad_Ly
    rx, ry = gamepad_Rx, gamepad_Ry
    gamepad_lock.release()
    return ly, lx, ry, rx


def yaw_actuation(joypos):
    GPIO.output(enable, off)
    stepTracker = 0
    delay = 0.0
    if joypos > 10:
        GPIO.output(enable, on)
        GPIO.output(direc, CW)
        delay = 0.01 + 0.01 * ((127.5 - joypos) / 127.5)
        stepTracker = 1
    elif joypos < -10:
        GPIO.output(enable, on)
        GPIO.output(direc, CCW)
        delay = 0.01 + 0.01 * ((127.5 + joypos) / 127.5)
        stepTracker = -1

    GPIO.output(step, GPIO.HIGH)
    time.sleep(delay)
    GPIO.output(step, GPIO.LOW)
    time.sleep(delay)
    return stepTracker


def goHome(steps):
    GPIO.output(enable, on)
    if steps >= 0:
        for i in range(0, steps):
            GPIO.output(direc, CCW)
            GPIO.output(step, GPIO.HIGH)
            time.sleep(0.01)
            GPIO.output(step, GPIO.LOW)
            time.sleep(0.01)
    else:
        for i in range(0, -1 * steps):
            GPIO.output(direc, CW)
            GPIO.output(step, GPIO.HIGH)
            time.sleep(0.01)
            GPIO.output(step, GPIO.LOW)
            time.sleep(0.01)


def chassisForward():
    Lspeed = 11.4 + 1
    Rspeed = 11.4 - 1
    Lmotor.ChangeDutyCycle(Lspeed)
    Rmotor.ChangeDutyCycle(Rspeed)


def chassisBackward():
    Lspeed = 11.4 - 1
    Rspeed = 11.4 + 1
    Lmotor.ChangeDutyCycle(Lspeed)
    Rmotor.ChangeDutyCycle(Rspeed)


def chassisClockwise():
    Lspeed = 11.4 + 1
    Rspeed = 11.4 + 1
    Lmotor.ChangeDutyCycle(Lspeed)
    Rmotor.ChangeDutyCycle(Rspeed)


def chassisCounterclockwise():
    Lspeed = 11.4 - 1
    Rspeed = 11.4 - 1
    Lmotor.ChangeDutyCycle(Lspeed)
    Rmotor.ChangeDutyCycle(Rspeed)

def chassisStop():
    Lspeed = 11.4
    Rspeed = 11.4
    Lmotor.ChangeDutyCycle(Lspeed)
    Rmotor.ChangeDutyCycle(Rspeed)


def chassisMove(X, Y):
    linear = 1.0 * (X / 127.5)
    angular = 1.0 * (Y / 127.5)

    Lspeed = 11.4 + linear + angular
    Rspeed = 11.4 - linear + angular
    Lmotor.ChangeDutyCycle(Lspeed)
    Rmotor.ChangeDutyCycle(Rspeed)



def cleanup(angle):
    global gamepad_run
    gamepad_run = False
    print(angle * (1.8 / 2.4))
    goHome(angle)
    GPIO.cleanup()
    print("Done")


def start():
    angleTracker = 0.0
    threading.Thread(target=gamepad_loop).start()
    Lmotor.start(11.4)
    Rmotor.start(11.4)

    while not gamepad_B:
        (ly, lx, ry, rx) = get_gamepad_input()
        # print("Left Joystick (Lx,Ly) is:\t(%s,%s)" % (lx, ly))
        # print("Right Joystick (Rx, Ry) is:\t(%s,%s)" % (rx, ry))

        motion = yaw_actuation(rx)
        angleTracker += (1.8 / 2.4) * motion
        if angleTracker > 360:
            angleTracker -= 360
        elif angleTracker < -360:
            angleTracker += 360

        if gamepad_back:
            angleTracker = 0.0

        if abs(ly) > 10 or abs(lx) > 10:
            chassisMove(lx, ly)
        else:
            chassisStop()


    cleanup(int(angleTracker / (1.8 / 2.4)))


start()
