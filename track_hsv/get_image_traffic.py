#! /usr/bin/env python 
from __future__ import division
import rospy 
from std_msgs.msg import Float32
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import math
import imutils
import numpy as np
import os
import time

# from keras.models import load_model
# import util
# import numpy as np

rospy.init_node("node_main",anonymous=True)
bridge =  CvBridge()
image = None
debug = False

def image_callback(msg):
    global image
    image = bridge.compressed_imgmsg_to_cv2(msg,'bgr8')

def nothing(x):
    pass
    
rospy.Subscriber('/camera/rgb/image_raw/compressed',CompressedImage,image_callback)
# modelPath =  "/home/p2h/catkin_ws/src/p2h/src/model/model_36-1.00.h5"
# model = load_model(modelPath)
# model.summary()

# pub_speed = rospy.Publisher('/teamp2h/set_speed',Float32,queue_size=1)
# pub_angle = rospy.Publisher('/teamp2h/set_angle',Float32,queue_size=1)
while not rospy.is_shutdown():
    if image is not None:   
        frame = image.copy()    
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        low_range = np.array([81,159,30])
        high_range = np.array([131,255,175])
        mask = cv2.inRange(hsv, low_range, high_range)

        # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))
        # mask = cv2.dilate(mask, np.ones((3,3), np.uint8), iterations=1)
        
        contours = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)[-2]
        contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]

        if len(contour_sizes) > 0:
            biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]
            area = cv2.contourArea(biggest_contour)
            if(area > 400):
                x,y,w,h = cv2.boundingRect(biggest_contour)
                if (abs(w-h) < (w+h)/10):
                    img = frame[y:y+h,x:x+w]
                    cv2.imshow('img',img)

        cv2.imshow('output', mask)
        # # countNonZero = np.count_nonzero(mask)
        # # print(countNonZero)
        cv2.imshow('frame', frame)   
    key = cv2.waitKey(1)
    if key == ord('c'):
        # i += 1
        cv2.imwrite("/home/p2h/catkin_ws/src/lhu_irc/src/node/data_traffic/"+str(time.time())+".jpg",img)
        print('/home/p2h/catkin_ws/src/lhu_irc/src/node/data_traffic/' + str(time.time())+ ".jpg")
    
    if key == ord('v'):
        # i += 1
        cv2.imwrite("/home/p2h/Desktop/track_hsv/"+str(time.time())+".jpg",frame)
        print('/home/p2h/Desktop/track_hsv/' + str(time.time())+ ".jpg")

    