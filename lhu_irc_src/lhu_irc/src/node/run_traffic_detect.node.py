#! /usr/bin/env python 
from __future__ import division
import rospy 
from std_msgs.msg import Float32
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import math
import os
# from lib.traffic import Traffic
from lib.traffic_detect_src import TrafficDetect

import numpy as np

rospy.init_node("node_main",anonymous=True)
bridge =  CvBridge()
image = None
# tf = Traffic()
tf = TrafficDetect()


def image_callback(msg):
    global image
    image = bridge.compressed_imgmsg_to_cv2(msg,'bgr8')


rospy.Subscriber('/camera/rgb/image_raw/compressed',CompressedImage,image_callback)

publish_traffic = rospy.Publisher('/traffic_detect', String, queue_size=1)

print('Traffic sign running!!!!')
while not rospy.is_shutdown():
  if image is not None:  
    frame = image.copy() 
    # res_tf_sign,mask = tf.detect_traffic_sign(frame)
    res_tf_sign = tf.process_image(frame)
    # cv2.cvtColor(frame,cv2.COLOR_GRay)
    # print(res_tf_sign)
    # cv2.imshow('output', mask)
    # cv2.waitKey(1)
    # publish_traffic.publish(res_tf_sign)
    # print(res_tf_sign)
    rospy.sleep(0.01)
    
    
    