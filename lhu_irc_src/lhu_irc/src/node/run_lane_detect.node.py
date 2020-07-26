#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32, Int32, Float32MultiArray, Bool
from cv_bridge import CvBridge
from lhu_irc.msg import Line

import cv2
import numpy as np

from lib.Config import Config
from lib.cardriver import CarDriver
from lib.lanedetect import LaneDetect
import time

config = Config()
config = config.get()

bridge = CvBridge()
cd = CarDriver("/car_steer_tmp", "/car_steer_tmp")
ld = LaneDetect(config)
frame = None
obstacle = 0
ss_status = False

#----------------------------- MAIN ---------------------------------#
rospy.init_node('run_lane_detect', anonymous=True)

def callback_stream(ros_data):
    global bridge, frame
    frame = bridge.imgmsg_to_cv2(ros_data, "bgr8")

def cb_obtascle_detect(ros_data):
    global obstacle
    obstacle = ros_data.data

def cb_ss(ros_data):
    global ss_status
    ss_status = ros_data.data

######################################################################################
pub = rospy.Publisher('/mid_lane', Int32, queue_size=10)
pub_angle_speed = rospy.Publisher('/angle_speed', Line, queue_size=1)

rospy.Subscriber("/camera/rgb/image_rect_color", Image, callback_stream)


# rospy.Subscriber("/bt2_status", Bool, cb_bt2)
# rospy.Subscriber("/bt3_status", Bool, cb_bt3)

while not rospy.is_shutdown():
    if(config['debug_time_lane'] and config['turn_off_all_debug']):
        start = time.time()

    if frame is not None:
        data_arr = ld.process_lane_detect(frame, obstacle)
        line = Line()
        line.data = data_arr
        print(line.data)
        # data_arr = Float32MultiArray(layout='',data=[data_arr])
        # pub_angle_speed.publish(line)

    if(config['debug_time_lane'] and config['turn_off_all_debug']):
        print('thoi gian xu ly lane', time.time() - start)

    rospy.sleep(0.001)