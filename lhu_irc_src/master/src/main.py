#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from cv_bridge import CvBridge
from lib.Car import get_car
from std_msgs.msg import Float32, Bool, String, Float32MultiArray, Int32
from master.msg import Line
from lib.yard import Yard
from time import time, sleep
from lib.function import exitFunction, resetTime, delayTime, reset_sObstatle
import san_trai, san_phai, test_san
import collections

bridge = CvBridge()
car = get_car()
frame = None
yard = Yard()
t = time()
s = False
loop = False
count_tfs = 0
temp_tfs = collections.deque(maxlen=10)

rospy.init_node('node_master_control', anonymous=True)

def myhook():
  print("Shutting down...")
  car.car_setLed(False)
  car.lcd.clean_lcd()

def init():
    sleep(1.5)
    car.lcd.set_lcd_1("San: {}".format(yard.san))
    car.lcd.set_lcd_2("Ham: {}".format(yard.ham))

rospy.on_shutdown(myhook)

############################################################## CALLBACK ##############################################################


def callback_line(ros_data):
    car.carval.line_left_angle = ros_data.left[0]
    car.carval.line_mid_angle = ros_data.mid[0]
    car.carval.line_right_angle = ros_data.right[0]
    car.carval.line_dotted_angle = ros_data.dotted[0]

    car.carval.line_left_curve = ros_data.left[1]
    car.carval.line_mid_curve = ros_data.mid[1]
    car.carval.line_right_curve = ros_data.right[1]
    car.carval.line_dotted_curve = ros_data.dotted[1]

    car.carval.line_left_distance = ros_data.left[2]
    car.carval.line_mid_distance = ros_data.mid[2]
    car.carval.line_right_distance = ros_data.right[2]
    car.carval.line_dotted_distance = ros_data.dotted[2]

def callback_ss(ros_data):  # Cảm biến
    car.carval.ss_status = ros_data.data


def callback_bt1(ros_data):  # Reset
    global s, t
    if ros_data.data:
        car.reset()
        s = False
        t = time()
        


def callback_bt2(ros_data):  # Chuyển sân
    if ros_data.data:
        yard.chuyen_san()
        car.lcd.set_lcd_1("San: {}".format(yard.san))
        car.lcd.set_lcd_2("Ham: {}".format(yard.ham))


def callback_bt3(ros_data):  # Chuyển hàm
    if ros_data.data:
        yard.chuyen_ham()
        car.lcd.set_lcd_2("Ham: {}".format(yard.ham))


def callback_bt4(ros_data):  # Start
    if ros_data.data:
        car.carval.is_running = True


def callback_parking(ros_data):
    car.carval.parking = ros_data.data



def callback_obstacle(ros_data):
    car.carval.obstacle = ros_data.data


def callback_traffic_sign(ros_data):
    global count_tfs, temp_tfs
    temp_tfs.append(ros_data.data)
    
    if temp_tfs.count('turn') >=5:
        car.carval.traffic_sign = 'turn'
        car.lcd.set_lcd_3("TFS: turn")
    if temp_tfs.count('straight') >=5:
        car.carval.traffic_sign = 'straight'
        car.lcd.set_lcd_3("TFS: straight")

def callback_bienbaocam(ros_data):
    car.carval.bien_bao_cam = ros_data.data

def callback_imu(ros_data):
    car.carval.imu_angle = ros_data.data

############################################################## SUBSCRIBER ##############################################################

#################### CAR HAL SUBSCRIBER ####################
rospy.Subscriber("/ss_status", Bool, callback_ss)
rospy.Subscriber("/bt1_status", Bool, callback_bt1)
rospy.Subscriber("/bt2_status", Bool, callback_bt2)
rospy.Subscriber("/bt3_status", Bool, callback_bt3)
rospy.Subscriber("/bt4_status", Bool, callback_bt4)
rospy.Subscriber("/car_yaw", Float32, callback_imu)

#################### DETECTED CALLBACK  ####################
rospy.Subscriber("/detect/lane", Line, callback_line)
rospy.Subscriber("/detect/lane/parking", Int32, callback_parking)
rospy.Subscriber("/detect/obstacle", Int32, callback_obstacle)
rospy.Subscriber("/detect/traffic_sign/signal", String, callback_traffic_sign)
rospy.Subscriber("/no_entry", Int32, callback_bienbaocam)

############################################################## PUBLISHER ##############################################################

car.pub_steer = rospy.Publisher("/set_steer_car_api", Float32, queue_size=1)
car.pub_speed = rospy.Publisher("/set_speed_car_api", Float32, queue_size=1)
car.pub_led = rospy.Publisher("/led_status", Bool, queue_size=1)
car.lcd.pub_lcd1 = rospy.Publisher("/lcd/line1", String, queue_size=1)
car.lcd.pub_lcd2 = rospy.Publisher("/lcd/line2", String, queue_size=1)
car.lcd.pub_lcd3 = rospy.Publisher("/lcd/line3", String, queue_size=1)
car.lcd.pub_lcd4 = rospy.Publisher("/lcd/line4", String, queue_size=1)
car.lcd.pub_clear = rospy.Publisher("/lcd/clear", Int32, queue_size=1)
car.pub_reset_imu = rospy.Publisher("/reset_imu", Bool, queue_size=1)


############################################################## MAIN ##############################################################

init()
print("Node Master Control Ready!")

while not rospy.is_shutdown():
    if car.carval.is_running:
        car.car_resetImu()
        temp_tfs.clear()
        reset_sObstatle()
        car.car_setLed(True)

        if yard.san == 'Left':
            if yard.ham == 'Exam':
                san_trai.chay_san_trai()
        if yard.san == 'Right':
            if yard.ham == 'Exam':
                san_phai.chay_san_phai()

        if yard.san == 'Test':
            if yard.ham == 'Climb Left':
                test_san.bam_trai(speed=11)
            if yard.ham == 'Climb Mid':
                test_san.bam_giua(speed=11)
            if yard.ham == 'Climb Right':
                test_san.bam_phai(speed=11)
            if yard.ham == 'Switch To Left':
                test_san.right_to_left()
            if yard.ham == 'Switch To Right':
                test_san.left_to_right()

        car.car_Brake()

        resetTime()        

        if loop:
            # ------- Dừng 3s theo yêu cầu BTC ------- #
            while delayTime(3):
                if exitFunction():
                    car.carval.is_running = False
                    car.car_setLed(False)
                    break
                sleep(0.01)
                car.car_setLed(False)
        else:
            car.carval.is_running = False
            car.car_setLed(False)


    else:
        if time() - t > 0.5:
            t = time()
            s = not s
            car.car_setLed(s)

    rospy.sleep(0.01)