#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from cv_bridge import CvBridge
from lib.Car import get_car
import cv2, os
from sensor_msgs.msg import Image
from datetime import datetime


bridge = CvBridge()
car = get_car()
frame,depth_frame = None, None

rospy.init_node('node_cap_data', anonymous=True)

############################################################## MAIN ##############################################################

### Capture data ###
now = datetime.now()
date_time = now.strftime("%d%m_%H%M%S")
fourcc = cv2.VideoWriter_fourcc(*"MJPG")
out = cv2.VideoWriter(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'logs', date_time + '.avi'), fourcc, 60.0, (320,240))
out_depth = cv2.VideoWriter(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'logs', date_time + '.avi'), fourcc, 60.0, (320,240))

def callback_camera(ros_data):
    global frame
    frame = bridge.imgmsg_to_cv2(ros_data, "bgr8")
    out.write(frame)

def callback_depth(ros_data):
    global depth_frame
    depth_frame = bridge.imgmsg_to_cv2(ros_data,"8UC1")
    out_depth.write(depth_frame)

rospy.Subscriber("/camera/rgb/image_rect_color", Image, callback_camera)
rospy.Subscriber("/camera/depth/image_raw", Image, callback_depth)

print("Node Master Control Ready!")

rospy.spin()

