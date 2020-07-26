#!/usr/bin/env python
# -*- coding: utf-8 -*-
from time import time, sleep
from Car import get_car
from datetime import datetime

rImu_angle = 0
rTime = time()
car = get_car()
lTime = time()
sObstacle = 0

# obstace_state = 0

# def check_obstacle():

def print_log(log):
    global lTime
    now = datetime.now()
    date_time = now.strftime("%d/%m %H:%M:%S.%f")   
    info = "{}: {} +{}ms\n".format(date_time, log, round(time() - lTime, 2))
    lTime = time()
    print(info)

def resetTime():
    global rTime
    rTime = time()


def delayTime(t):
    if time() - rTime > t:
        return False
    return True

def reset_sObstatle():
    global sObstacle
    sObstacle = 0

def car_ThayVatCan():
    global sObstacle
    if sObstacle == 0:
        print_log("Thay vat can")
        sObstacle = 1
        resetTime()

def car_NeVatCan():
    global sObstacle
    if sObstacle == 1:
        if car.carval.line_dotted_angle < 50:
            car.car_ChayTheoGoc(speed=13, steer=-45)
        else: 
            if car.car_CoLineTrai():
                car.car_BamTrai(speed=13, defAng=5)
            else:
                car.car_ChayTheoGoc(speed=13, steer=60)
        if not delayTime(2.5):
            sObstacle = 2
            resetTime()
        print_log("Sang trai")
        return True   
    if sObstacle == 2:
        print_log("Sang phai")
        carSwitchToRightLane(speed=13)
        sObstacle = 3
        return True
    return False


def resetImuAngle():
    global rImu_angle
    rImu_angle = car.carval.imu_angle


def car_CuaTheoGocImu(imu_angle, speed = 10, car_steer=None):
    T = car.carval.imu_angle - rImu_angle
    car.car_Forward(speed)
    if car_steer is None:
        car.set_Steer(T * 3)
    else:
        car.set_Steer(car_steer)

    if abs(T + 5) > imu_angle:
        return False
    else:
        return True


def exitFunction():
    # Trạng thái cho chạy & Cảm biến không bị che#
    if car.carval.is_running and car.carval.ss_status:
        return False
    return True


def waitSensorRelease():
    while not car.carval.ss_status:
        sleep(0.01)
    sleep(0.5)


# =====Chay theo imu mot goc============================================================================
def carRun_imu(target_imu, speed=10, car_steer=None):
    T = target_imu - car.carval.imu_angle
    car.car_Forward(speed)
    if car_steer is None:
        car.set_Steer(T * 3)
    else:
        car.set_Steer(car_steer)

    if abs(T) < 5:
        return False
    else:
        return True

def chay_goc_cua_90(line='right', speed = 12, imu_angle=70, car_steer=60, defAng=-20, curve = 35):
    if line == 'right':
        while car.car_CoLinePhai() and car.carval.line_right_slope < curve:
            if exitFunction():
                return
            car.car_BamPhai(speed=16, defAng=defAng)
            sleep(0.01)

        resetImuAngle()
        while car_CuaTheoGocImu(imu_angle=imu_angle, speed=speed, car_steer=car_steer):
            if exitFunction():
                return
            sleep(0.01)

def carSwitchToLeftLane(speed = 12, target_dotted_angle = 60, car_steer = -45):
    while car.carval.line_dotted_angle < target_dotted_angle:
        car.car_ChayTheoGoc(speed=speed, steer=car_steer)
        if exitFunction():
            return
        sleep(0.01)

def carSwitchToRightLane(speed = 12, target_dotted_angle = -60, car_steer = 45):
    while car.carval.line_dotted_angle < target_dotted_angle:
        car.car_ChayTheoGoc(speed=speed, steer=car_steer)
        if exitFunction():
            return
        sleep(0.01)


