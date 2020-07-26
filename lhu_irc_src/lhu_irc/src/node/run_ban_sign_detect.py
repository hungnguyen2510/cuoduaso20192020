#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32, Int32, Int32MultiArray, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import imutils
from lib.bans_tfsign import BansTFSign
import time

bridge = CvBridge()
bans = BansTFSign()

frame = None
depth_frame = None

#----------------------------- MAIN ---------------------------------#
rospy.init_node('run_obtascle_detect', anonymous=True)

def callback_stream(ros_data):
    global bridge, frame
    frame = bridge.imgmsg_to_cv2(ros_data, "bgr8")

def callback_depth(ros_data):
    global bridge, depth_frame
    depth_frame = bridge.imgmsg_to_cv2(ros_data,"8UC1")

######################################################################################
rospy.Subscriber("/camera/rgb/image_rect_color", Image, callback_stream)
rospy.Subscriber("/camera/depth/image_raw", Image, callback_depth)
pub_bans_sign = rospy.Publisher('/obstacle_detect', Int32, queue_size=10)

while not rospy.is_shutdown():
    if frame is not None and depth_frame is not None:
        bans_sign = bans.detect_red_sign(frame.copy(),depth_frame.copy()) #MID
        pub_bans_sign.publish(obstacle)
    rospy.sleep(0.001)