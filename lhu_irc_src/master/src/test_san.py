#!/usr/bin/env python
# -*- coding: utf-8 -*-
from lib.Car import get_car
from time import time, sleep
from lib.function import exitFunction, resetTime, delayTime, carSwitchToLeftLane, carSwitchToRightLane

car = get_car()

def bam_trai(speed=12):
   while 1:
        if exitFunction():
                return
        car.set_Steer(car.carval.line_left_angle)
        car.car_Forward(speed)
        sleep(0.01)

def bam_phai(speed=12):
    while 1:
        if exitFunction():
                return
        car.car_BamPhai(speed=speed, defAng=0)
        sleep(0.01)

def bam_giua(speed=12):
    while 1:
        if exitFunction():
                return
        car.car_BamGiua(speed=speed)
        sleep(0.01)

def right_to_left(speed=15):
    resetTime()
    while delayTime(1):
        if exitFunction():
            return
        car.car_BamPhai(speed=speed)

    carSwitchToLeftLane(speed=speed)

    resetTime()
    while delayTime(2):
        if exitFunction():
            return
        car.car_BamTrai(speed=speed)

    carSwitchToRightLane(speed=speed)

    resetTime()
    while delayTime(3):
        if exitFunction():
            return
        car.car_BamPhai(speed=speed)

def left_to_right(speed=13):
    resetTime()
    while delayTime(1):
        if exitFunction():
            return
        car.car_BamTrai(speed=speed)

    carSwitchToRightLane(speed=speed)

    resetTime()
    while delayTime(2):
        if exitFunction():
            return
        car.car_BamPhai(speed=speed)
