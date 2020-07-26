#!/usr/bin/env python
# -*- coding: utf-8 -*-

from lib.Car import get_car
from lib.function import exitFunction, resetTime, delayTime, waitSensorRelease, resetImuAngle, carRun_imu, car_CuaTheoGocImu, print_log
from time import sleep

car = get_car()

def chu_trinh_5_1():
    # -------- Chờ phất cờ -------- #
    waitSensorRelease()


    print_log("chu_trinh_5_1: Chay thang bam phai")


    resetTime()

    while delayTime(3.3):
        if exitFunction():
            print_log('Exit')
            return
        if car.car_CoLinePhai() and car.carval.line_right_angle < 5:
            car.car_BamPhai(speed=20, num=-40, division=5)

        else:
            carRun_imu(target_imu=0, speed=18)
            
        sleep(0.01)

    while car.car_CoLineTrai():
        if exitFunction():
            print_log('Exit')
            return
        car.car_BamPhai(speed=18, num=-30)          
        sleep(0.01)

    print_log("chu_trinh_5_1: Re trai")
    resetTime()
    while carRun_imu(target_imu=-60, speed=15, car_steer=-60):
        if exitFunction():
            print_log('Exit')
            return
        sleep(0.01)
    
    # -------- Chạy thẳng 0.22s cho xe nằm thẳng với đường để chạy mù qua tuyết -------- #
    resetTime()
    while delayTime(0.22):
        if exitFunction():
            print_log('Exit')
            return
        car.car_BamGiua(speed=15)
        sleep(0.01)


def chu_trinh_1_2():
    # -------- Chạy mù qua tuyết 1s -------- #
    print_log("chu_trinh_1_2: Qua tuyet")
    resetTime()
    while delayTime(1.2):
        if exitFunction():
            print_log('Exit')
            return
        car.car_ChayTheoGoc(speed=15, steer=0)
        sleep(0.01)


def chu_trinh_2_5():
    # -------- Khi có line phải, bám giữa qua đường cong 2 - 5 -------- #

    print_log("chu_trinh_2_5: Bam giua")
    while car.carval.imu_angle > -190 or car.car_CoLinePhai():
        if exitFunction():
            return
        car.car_BamGiua(speed=18)
        sleep(0.01)

    print_log("chu_trinh_2_5: BamTrai")

    while car.car_CoLinePhai():
        if exitFunction():
            return
        car.car_BamTrai(speed=16, defAng=5)
        sleep(0.01)

    resetImuAngle()

    print_log("chu_trinh_2_5: Re phai")
    # -------- Rẽ phải 70 độ vào đường 2 chiều -------- #
    while car_CuaTheoGocImu(imu_angle=70, speed=15, car_steer=60):
        if exitFunction():
            return
        sleep(0.01)
    
    resetTime()


    print_log("chu_trinh_2_5: Bam phai bien bao")
    # -------- Bám phải đến biển báo -------- #
    while delayTime(1.2):
        if exitFunction():
            return
        if not delayTime(0.20) and car.carval.line_right_curve > 10:
            break
        car.car_BamPhai(speed = 12, defAng=-25)
        sleep(0.01)   

    print_log("chu_trinh_2_5: Chay mu")

    resetTime()
    while delayTime(0.2):
        if exitFunction():
            return
        car.car_ChayTheoGoc(speed = 12, steer = 0)
        sleep(0.01)   
    

def chu_trinh_5_3_4_5():

    print_log("vao chu trinh chu_trinh_5_3_4_5")

    resetTime()
    while delayTime(2):
        if exitFunction():
            return
        if not delayTime(1.5) and car.car_Co2Line():
            break
        car.car_ChayTheoGoc(speed=16, steer=50)
        sleep(0.01)

    
    while car.carval.imu_angle > -330:
        if exitFunction():
            return
        if car.carval.line_right_curve < -35:
            car.car_BamPhai(speed=18, num=car.carval.line_right_curve/2)
        else:
            car.car_BamPhai(speed=18, num=0, division=1, defAng=-5)
        sleep(0.01)

    print_log("chu_trinh_5_3_4_5: chay & kiem tra parking")    

    while car.carval.parking == -1:
        if exitFunction():
            return
        car.car_BamPhai(speed=20, num=-30)
        sleep(0.01)
    



def chay_san_trai_cham():
    chu_trinh_5_1()
    chu_trinh_1_2()
    chu_trinh_2_5()
    chu_trinh_5_3_4_5()

