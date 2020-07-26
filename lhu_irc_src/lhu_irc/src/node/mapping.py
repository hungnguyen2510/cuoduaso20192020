#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import cv2
import imutils
import numpy as np
from math import cos, sin, radians, pi

RANGE_AROUND = 3.0 #m
PART_AREA = [90,270]

lidar_range = 8.0
mapp_size = 3.0
image_size = [300, 300]

k_closing = np.ones((5,5), dtype=np.uint8)

def lidar_pose(src):
    pt1 = (150, 145)
    pt2 = (145, 155)
    pt3 = (155, 155)
    triangle_cnt = np.array([pt1, pt2, pt3])
    cv2.drawContours(src, [triangle_cnt], 0, (255,255,255), -1)

def count_nonblack_np(img):
    return img.any(axis=-1).sum()

def isObjectL(cnt):

    x, y, w, h = cv2.boundingRect(cnt)
    x1, x2, y1, y2, cX, cY = x, x+w, y, y+h, (x + (w//2)), (y+(h//2))

    dX = cv2.norm(x1 - x2)
    dY = cv2.norm(y1 - y2)
    dRect = abs(dX - dY)

    if dRect < dX and dRect < dY:
        return True

    return False

def object_finding(src, arr):
    global pub_object
    gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 0, 255, 0)

    # closing = cv2.dilate(thresh,k_closing,iterations=1)
    closing = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, k_closing)
    contours = cv2.findContours(closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)

    cv2.circle(src, (image_size[0]//2,image_size[1]//2), int(RANGE_AROUND*50), (215,150,100), 1)

    #rectangle
    start_point = (image_size[0]//2 - 30, image_size[0]//2 - 100)
    end_point = (image_size[1]//2 + 30, image_size[1]//2 + 100) 
    cv2.rectangle(src,start_point,end_point,(255,255,0),1)

    cropped_img = gray[start_point[1]:end_point[1], start_point[0]:end_point[0]]
    cnt_ranges = 0
    hasObject = False
    object_postition = ''

    hasObject = True if len(contours) > 0 else False

    w_car = start_point[0] + end_point[0]
    leftCarArea = [start_point[0],w_car//2]
    rightCarArea = [w_car//2,end_point[0]]
        
    for contour in contours:
        obstacle_size = cv2.contourArea(contour)
        if obstacle_size > 40:
            x,y,w,h = cv2.boundingRect(contour)
            x1, x2, y1, y2, cX, cY = x, x+w, y, y+h, (x + (w//2)), (y + (h//2))

            if (start_point[0] < cX < end_point[0]) and (start_point[1] < cY < end_point[1]):

                if leftCarArea[0] < cX < leftCarArea[1]:
                    object_postition = 'left'
                if rightCarArea[0] < cX < rightCarArea[1]:
                    object_postition = 'right'

            if isObjectL(contour) :
                cv2.drawContours(src, [contour], -1, (0, 255, 0), 2)

            cv2.rectangle(src,(x1,y1),(x2,y2),(0,255,0),1)

    cv2.imshow("gray", closing)
    cv2.imshow('corp',cropped_img)

    if count_nonblack_np(cropped_img) != 0:
        pub_object.publish(object_postition)
        print("Nguy hiem !")

    if count_nonblack_np(cropped_img) == 0 and hasObject == True:
        print('Canh bao !')
    
    if hasObject == False:
        print('Khong co vat can !')

def main_process(arr):
    arr_range = []
    maps = np.zeros((image_size[0]//2,image_size[1],3), dtype=np.uint8)
    x_center, y_center = image_size[0]//2, image_size[1]//2

    w = image_size[0]//2
    h = image_size[1]//2

    for i in range(0,len(arr)):
        if str(arr[i]) == 'inf' or arr[i] > lidar_range:
            continue

        yy = cos(radians(i)) * arr[i]*(w/mapp_size)
        xx = sin(radians(180+i)) * arr[i]*(h/mapp_size)
        if arr[i] < RANGE_AROUND: # and (i < PART_AREA[0] or i > PART_AREA[1]):
            x = x_center + xx
            y = y_center - yy
            arr_range.append([[x,y],arr[i]])
            cv2.circle(maps, (int(x),int(y)), 2, (0,0,255), 3)

    object_finding(maps, arr_range)
    lidar_pose(maps)

    cv2.imshow("maps", maps)
    cv2.waitKey(1)

def scan_callback(msg):
    main_process(msg.ranges) 

rospy.init_node('scan_around')
rospy.Subscriber('/scan', LaserScan, scan_callback)

pub_object = rospy.Publisher('sensor/object', String, queue_size=1)

rospy.spin()