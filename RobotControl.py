#!/usr/bin/env python3
# # # # # # # # # # # # # # # # # #
# File: RobotControl.py
# Author: Jake Erichson
# UR ME205 Wheel Code
# Last Revision: 4/29/18
# # # # # # # # # # # # # # # # # #

# To Update Through GitHub:
# Mac Terminal: cd ~/PycharmProjects/ME205_RC
#               git add -u
#               git commit -m "______"
#               git push -u origin master
# To Pull:
# Rpi Terminal: cd /home/pi/ME205_RobotControl
#               git reset --hard
#               git pull
#               chmod 755 RobotControl.py
# To See Status: git status
# To Add a new file: git add {path to file}

import threading
import time

# Delay start up to avoid "ghost" motion and allow gamepad connection
time.sleep(30)

from inputs import get_gamepad
import RPi.GPIO as GPIO
from subprocess import call

# Initialize all the gamepad variables
gamepad_lock = threading.Lock()
gamepad_Lx = 0.0
gamepad_Ly = 0.0
gamepad_Rx = 0.0
gamepad_Ry = 0.0
gamepad_B = False
gamepad_back = False
gamepad_start = False
gamepad_A = False
gamepad_X = False
gamepad_Y = False
gamepad_Dx = 0.0
gamepad_Dy = 0.0
gamepad_Rt = False
gamepad_Lt = False
gamepad_run = True

# Set pin numbers and settings for motors
on = 0
off = 1
CW = 1
CCW = 0
direc = 25
enable = 23
step = 24
lCimPin = 12
rCimPin = 18

# Setup GPIO pins correctly
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(enable, GPIO.OUT)
GPIO.setup(step, GPIO.OUT)
GPIO.setup(direc, GPIO.OUT)
GPIO.setup(lCimPin, GPIO.OUT)
GPIO.setup(rCimPin, GPIO.OUT)
Lmotor = GPIO.PWM(lCimPin, 80)
Rmotor = GPIO.PWM(rCimPin, 80)
Lmotor.start(11.4)
Rmotor.start(11.4)


def gamepad_loop():
    # to get all gamepad info in separate threads
    global gamepad_Lx
    global gamepad_Ly
    global gamepad_Rx
    global gamepad_Ry
    global gamepad_B
    global gamepad_back
    global gamepad_start
    global gamepad_A
    global gamepad_X
    global gamepad_Y
    global gamepad_Dx
    global gamepad_Dy
    global gamepad_Rt
    global gamepad_Lt

    while gamepad_run:
        events = get_gamepad()
        for event in events:
            # print(event.code)
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
            if str(event.code) == "BTN_TR2":
                gamepad_lock.acquire()
                gamepad_start = event.state
                gamepad_lock.release()
            if str(event.code) == "BTN_EAST":
                gamepad_lock.acquire()
                gamepad_A = event.state
                gamepad_lock.release()
            if str(event.code) == "BTN_SOUTH":
                gamepad_lock.acquire()
                gamepad_X = event.state
                gamepad_lock.release()
            if str(event.code) == "BTN_NORTH":
                gamepad_lock.acquire()
                gamepad_Y = event.state
                gamepad_lock.release()
            if str(event.code) == "ABS_HAT0X":
                gamepad_lock.acquire()
                gamepad_Dx = event.state
                gamepad_lock.release()
            if str(event.code) == "ABS_HAT0Y":
                gamepad_lock.acquire()
                gamepad_Dy = event.state
                gamepad_lock.release()
            if str(event.code) == "BTN_TR":
                gamepad_lock.acquire()
                gamepad_Rt = event.state
                gamepad_lock.release()
            if str(event.code) == "BTN_TL":
                gamepad_lock.acquire()
                gamepad_Lt = event.state
                gamepad_lock.release()


def get_gamepad_input():
    # to return certain gamepad inputs
    gamepad_lock.acquire()
    lx, ly = gamepad_Lx, gamepad_Ly
    rx, ry = gamepad_Rx, gamepad_Ry
    gamepad_lock.release()
    return ly, lx, ry, rx


def yaw_actuation(joypos):
    # to move stepper at variable speeds in either direction
    GPIO.output(enable, off)
    stepTracker = 0
    delay = 0.0
    if joypos > 10:
        GPIO.output(enable, on)
        GPIO.output(direc, CW)
        delay = 0.0037 + 0.0075 * ((127.5 - joypos) / 127.5)
        stepTracker = 1
    elif joypos < -10:
        GPIO.output(enable, on)
        GPIO.output(direc, CCW)
        delay = 0.00365 + 0.0075 * ((127.5 + joypos) / 127.5)
        stepTracker = -1

    GPIO.output(step, GPIO.HIGH)
    time.sleep(delay)
    GPIO.output(step, GPIO.LOW)
    time.sleep(delay)
    GPIO.output(enable, off)
    return stepTracker


def goHome(steps):
    # to move stepper back to zero position
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
    # to go forward
    Lspeed = 11.45 + 1.5
    Rspeed = 11.45 - 1.5
    Lmotor.ChangeDutyCycle(Lspeed)
    Rmotor.ChangeDutyCycle(Rspeed)


def chassisBackward():
    # to go backward
    Lspeed = 11.45 - 1.5
    Rspeed = 11.45 + 1.5
    Lmotor.ChangeDutyCycle(Lspeed)
    Rmotor.ChangeDutyCycle(Rspeed)


def chassisClockwise():
    # to spin clockwise
    Lspeed = 11.45 + 1.5
    Rspeed = 11.45 + 1.5
    Lmotor.ChangeDutyCycle(Lspeed)
    Rmotor.ChangeDutyCycle(Rspeed)


def chassisCounterclockwise():
    # to spin counterclockwise
    Lspeed = 11.45 - 1.5
    Rspeed = 11.45 - 1.5
    Lmotor.ChangeDutyCycle(Lspeed)
    Rmotor.ChangeDutyCycle(Rspeed)


def chassisStop():
    # to stop both wheels
    Lspeed = 11.45
    Rspeed = 11.45
    Lmotor.ChangeDutyCycle(Lspeed)
    Rmotor.ChangeDutyCycle(Rspeed)


def chassisMove(X, Y):
    # to move the chassis based on desired
    #   linear and angular velocities
    linear = (Y / 127.5)
    angular = (X / 127.5)

    maxSpeed = 4.0  # 0.1 to 7.1
    Lspeed = 11.45 + maxSpeed*linear + maxSpeed*angular
    Rspeed = 11.45 - maxSpeed*linear + maxSpeed*angular
    Lmotor.ChangeDutyCycle(Lspeed)
    Rmotor.ChangeDutyCycle(Rspeed)

    # to return wheel velocities
    Lvel = 1.0639*(Lspeed-11.4) - 0.2668
    Rvel = 1.0639*(Rspeed-11.4) - 0.2668

    return Lvel, Rvel


def robotSpin(direction):
    # to spin chassis and counter-rotate top
    # direction = (-1: CCW, 1: CW, 0: none)
    yaw_actuation(-127.5*direction)
    Lspeed = 11.45 + 1.5*direction
    Rspeed = 11.45 + 1.5*direction
    Lmotor.ChangeDutyCycle(Lspeed)
    Rmotor.ChangeDutyCycle(Rspeed)


def cleanup(angle):
    # to reset GPIO pins, call goHome(), and
    #   shutdown the raspberry pi safely
    global gamepad_run
    print(angle * (1.8 / 2.4))
    gamepad_run = False
    goHome(angle)
    GPIO.cleanup()
    print("Goodbye")
    call("sudo shutdown -t now", shell=True)


def start():
    # main control function to run the Wheelchair
    threading.Thread(target=gamepad_loop).start()

    # loops until shutoff by RT and LT, together
    while not gamepad_Rt and not gamepad_Lt:
        angleTracker = 0.0

        # drive system engaged with A
        if gamepad_A:
            print("Start")
            # drive system cuts out with B
            while not gamepad_B:
                (ly, lx, ry, rx) = get_gamepad_input()
                # print("Left Joystick (Lx,Ly) is:\t(%s,%s)" % (lx, ly))
                # print("Right Joystick (Rx, Ry) is:\t(%s,%s)" % (rx, ry))

                # move the person (tolerance in function)
                motion = yaw_actuation(rx)

                # keep track of angle difference, chassis vs. person
                angleTracker += (1.8 / 2.4) * motion
                if angleTracker > 360:
                    angleTracker -= 360
                elif angleTracker < -360:
                    angleTracker += 360

                # zero the angle difference
                if gamepad_back:
                    angleTracker = 0.0

                # move the chassis if instructed
                if abs(ly) > 10 or abs(lx) > 10:
                    left_velocity, right_velocity = chassisMove(lx, ly)
                else:
                    chassisStop()
                    right_velocity = 0.0
                    left_velocity = 0.0

            print(angleTracker)
            print("Done")

        # hardcoded box run with X
        elif gamepad_X:
            chassisForward()
            time.sleep(3)
            chassisStop()
            time.sleep(1)
            for i in range (0, 105):
                robotSpin(1)
            chassisStop()
            time.sleep(1)
            chassisForward()
            time.sleep(1.5)
            chassisStop()
            time.sleep(1)
            for i in range (0, 100):
                robotSpin(-1)
            chassisStop()
            time.sleep(1)
            chassisBackward()
            time.sleep(3)
            chassisStop()
            time.sleep(1)
            for i in range (0, 100):
                robotSpin(-1)
            chassisStop()
            time.sleep(1)
            chassisForward()
            time.sleep(1.5)
            chassisStop()
            time.sleep(1)
            for i in range (0, 100):
                robotSpin(1)
            chassisStop()

        # hardcoded line run with Y
        elif gamepad_Y:
            chassisForward()
            time.sleep(4)
            chassisStop()
            time.sleep(1)
            for i in range(0, 430):
                robotSpin(1)
            chassisStop()
            time.sleep(1)
            chassisBackward()
            time.sleep(4)
            chassisStop()

        # hardcoded rotation and yaw counter with x-axis D-pad
        elif not gamepad_Dx == 0:
            while not gamepad_Dx == 0:
                robotSpin(gamepad_Dx)
            chassisStop()

        # hardcoded forward and backward motion with D-pad
        elif not gamepad_Dy == 0:
            while not gamepad_Dy == 0:
                if gamepad_Dy < 0:
                    chassisForward()
                elif gamepad_Dy > 0:
                    chassisBackward()
            chassisStop()

    cleanup(int(angleTracker / (1.8 / 2.4)))


if __name__ == '__main__':
    start()