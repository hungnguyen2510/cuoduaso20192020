#!/usr/bin/env python
# -*- coding: utf-8 -*-

from lib.Car import get_car
from lib.function import exitFunction, resetTime, delayTime, waitSensorRelease, resetImuAngle, carRun_imu, car_CuaTheoGocImu, print_log, car_ThayVatCan, car_NeVatCan
from time import sleep

car = get_car()


####################################################
################## chu_trinh_5_1 ###################
def chu_trinh_5_1():
    # -------- Chờ phất cờ -------- #
    waitSensorRelease()


    # -------- Bám phải, qua bóng tòa nhà, đến góc cua 90 -------- #

    while car.carval.line_right_curve > -17.5:
        sleep(0.01)
        if exitFunction():
            return
        if car.car_CoLinePhai():
            car.car_BamPhai(18, defAng=-20)
        else:
            car.car_ChayTheoGoc(18, 0)


    # --------- Rẽ trái đến khi thấy lane --------- #
    
    while not car.car_Co2Line():
        sleep(0.01)
        if exitFunction():
            return
        car.car_ChayTheoGoc(13, -60)

    resetTime()
    while delayTime(0.5):
        sleep(0.01)
        if exitFunction():
            return
        car.car_BamGiua(speed=15)


####################################################
################## chu_trinh_1_2 ###################
def chu_trinh_1_2():
    # -------- Chạy mù qua tuyết 1s -------- #
    resetTime()
    while delayTime(1.25):
        sleep(0.01)
        if exitFunction():
            return
        car.car_ChayTheoGoc(speed=15, steer=0)


####################################################
################## chu_trinh_2_5 ###################
def chu_trinh_2_5():

    # -------- Khi có line phải, bám giữa qua đường cong 2 - 5 -------- #
    
    while car.car_Co1Trong2Line():
        sleep(0.01)
        if exitFunction():
            return
        if car.car_CoLinePhai() and car.carval.line_right_curve > 35:
            break
        car.car_BamGiua(speed=18)

    resetTime()
    while delayTime(0.65) or not car.car_Co2Line() or (car.car_CoLinePhai() and car.carval.line_right_curve < 0):
        sleep(0.01)
        if exitFunction():
            return
        car.car_ChayTheoGoc(speed=15, steer=60)

    while car.car_CoLinePhai() and car.carval.line_right_curve < 50:
        sleep(0.01)
        if exitFunction():
            return
        car.car_BamPhai(speed=15, defAng=-5)
    

#################################################################    
############################ 5-3-4-5 ############################
#################################################################
def chu_trinh_5_3_4_5():

    print("chu_trinh_5_3_4_5: RIGHT")

    while car.carval.line_right_distance < 100:
        sleep(0.01)
        if exitFunction():
            return
        car.car_BamPhai(speed = 15, defAng=-10)

    while abs(car.carval.line_right_curve) > 20:
        car.car_ChayTheoGoc(speed=15, steer=50)
        sleep(0.01)
        if exitFunction():
            return
    
    print_log("while car.carval.parking == -1:")
    while car.carval.parking == -1:
        sleep(0.01)
        if exitFunction():
            return
        if not car.car_CoLinePhai():
            car.car_ChayTheoGoc(speed=17, steer=-60)
            continue
        car.car_BamPhai(speed = 18, defAng=-15)
    

#################################################################
############################ 5-4-3-5 ############################
#################################################################

def chu_trinh_5_4():
    print_log("Chu trinh 5_4")
    print_log("NGA 3 THU 1")
    # # ----------- NGÃ 3 THỨ 1 ----------- # 
    # chạy mù đến khi thấy 2 line #
    resetTime()
    while delayTime(0.6) or not car.car_Co2Line() or car.carval.line_right_distance > 120:
        sleep(0.01)
        if exitFunction():
            return
        car.car_ChayTheoGoc(speed=17, steer=-2)

    print_log("Bam phai")

    while car.car_CoLinePhai():
        sleep(0.01)
        if exitFunction():
            return
        if car.carval.line_right_curve > 45 and car.carval.line_right_distance < 160:
            break
        car.car_BamPhai(speed=17, defAng=-30)


def chu_trinh_5_4_3_5():

    # print_log("chu_trinh_5_4_3_5: STRAIGHT")
    # chu_trinh_5_4()
    
    # print_log("NGA 3 THU 2")
    # # ----------- NGÃ 3 THỨ 2 ----------- # 
    # # ----------- BẮT ĐẦU VẬT CẢN  ----------- # 

    # print_log("Chay mu")
    # resetTime()
    # while delayTime(0.7) or not car.car_CoLinePhai() or car.carval.line_right_distance > 120:
    #     sleep(0.01)
    #     if exitFunction():
    #         return
    #     car.car_ChayTheoGoc(speed=13, steer=0)

    while car.carval.bien_bao_cam != 1:
        sleep(0.01)
        if exitFunction():
            return
        if 0 < car.carval.obstacle < 55:
            car_ThayVatCan()
        if car_NeVatCan():
            continue
        if car.car_CoLinePhai() and car.carval.line_right_distance < 160:
            car.car_BamPhai(speed=13, defAng=-25)
        if car.carval.line_right_distance > 160 or car.carval.line_right_curve > 30:
            car.car_ChayTheoGoc(speed=13, steer=60)
    
    

#     # ----------- NGÃ 3 THỨ 3 ----------- # 
#     resetTime()
#     while delayTime(1.25) or not car.car_CoLinePhai():
#         sleep(0.01)
#         if exitFunction():
#             return
#         car.car_ChayTheoGoc(speed=13, steer=0)

#     # ----------- KẾT THÚC VẬT CẢN  ----------- # 

#     print_log("Bam phai")
#     while car.car_CoLinePhai():
#         sleep(0.01)
#         if exitFunction():
#             return
#         if car.carval.line_right_curve > 37.5 and car.carval.line_right_distance < 160:
#             break
#         car.car_BamPhai(speed=14, defAng=-35)
        
#     # ----------- Góc cua 3 ----------- # 
#     print_log("Goc cua 3")
#     resetTime()
#     while delayTime(1) or car.carval.line_right_distance > 80 or car.carval.line_right_curve > 15:
#         sleep(0.01)
#         if exitFunction():
#             return
#         car.car_ChayTheoGoc(speed=13, steer=60)

#     print_log("Bam phai")
#     while car.car_CoLinePhai():
#         sleep(0.01)
#         if exitFunction():
#             return
#         if car.carval.line_right_curve > 37.5 and car.carval.line_right_distance < 160:
#             break
#         car.car_BamPhai(speed=15, defAng=-20)

#    # ----------- NGÃ 3 THỨ 4 ----------- # 
#     resetTime()
#     while delayTime(1) or not car.car_Co2Line() or abs(car.carval.line_right_curve) > 20:
#         sleep(0.01)
#         if exitFunction():
#             return
#         car.car_ChayTheoGoc(speed=15, steer=-42)

#     print_log("while car.carval.parking == -1:")
#     while car.carval.parking == -1:
#         sleep(0.01)
#         if exitFunction():
#             return
#         car.car_BamPhai(speed = 15, defAng=-15)
        


#################################################################
############################ 2 CHIEU ############################
#################################################################
def chu_trinh_duong_2_chieu():
  
    print("Traffic Sign: {}".format(car.carval.traffic_sign))
    if car.carval.traffic_sign == 'turn':
        chu_trinh_5_3_4_5()
    else:
        chu_trinh_5_4_3_5()

def chay_san_trai():
    # chu_trinh_5_1()
    # chu_trinh_1_2()
    # chu_trinh_2_5()
    # chu_trinh_duong_2_chieu()
    # chu_trinh_5_3_4_5()
    chu_trinh_5_4_3_5()
    print_log("chay_san_trai")
    
    # while 1:
    #     sleep(0.01)
    #     if exitFunction():
    #         return
    #     if car.carval.line_right_curve > 30:
    #         car.car_ChayTheoGoc(speed=12, steer=60)
    #         continue
    #     car.car_BamPhai(speed=12, defAng=-10)

