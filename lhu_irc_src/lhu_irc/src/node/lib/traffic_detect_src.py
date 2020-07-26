#! /usr/bin/env python
import rospy
import cv2
import numpy as np
from math import acos
import time
import imutils
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Int32
from cv_bridge import CvBridge

class TrafficDetect:
    def __init__(self):
        self._debug = 1
        self.reprocess_tf = 0

        # self.low_range = np.array([80,114,75])
        # self.high_range = np.array([120,255,255])

        # self.low_range = np.array([80,101,65])
        # self.high_range = np.array([140,255,170])

        # self.low_range = np.array([107,130,51])
        # self.high_range = np.array([151,255,235])
        
        # self.low_range = np.array([81,159,30])
        # self.high_range = np.array([131,255,175])

        self.low_range = np.array([101,190,77])
        self.high_range = np.array([126,255,153])
        #blue_area_rect_size:
        self.min_area = 200
        self.max_area = 500
        
        # self.arrow_thr = 250
        self.thr_bin = 80
        # self.k_close = np.ones((5,5),np.uint8)
        self.k_open = np.ones((3,3),np.uint8)
    
    def blue_area(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        blue = cv2.inRange(hsv, self.low_range, self.high_range)
        # closing = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, np.ones(self.k_close))
        kernel = np.ones((5,6), np.uint8)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        opening = cv2.morphologyEx(blue, cv2.MORPH_OPEN, self.k_open)
        opening = cv2.dilate(opening, kernel, iterations=1)
        # opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, np.ones(self.k_open))
        if self._debug:
            cv2.imshow("extract blue", blue)
            # cv2.imshow("closing", closing)
            cv2.imshow("opening", opening)
            cv2.waitKey(1)
        return opening
    
    def direct_indentify(self, img):
        rgb = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
        cnts = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        angle = None
        if len(cnts) > 0:
            max_rect = None
            max_area = self.max_area
            min_area = self.min_area
            for cnt in cnts:
                area = cv2.contourArea(cnt)
                print(area)
                box = cv2.boxPoints(cv2.minAreaRect(cnt))
                box = np.int0(box)
                if area > min_area and area < max_area:
                    max_rect = box
                    # cv2.drawContours(rgb,cnt,0,(255,0,255),5)
            if max_rect is not None:
                edge1 = max_rect[1] - max_rect[0]
                edge2 = max_rect[2] - max_rect[1]
                longest_edge = edge1
                if cv2.norm(edge2) > cv2.norm(longest_edge):
                    longest_edge = edge2
                # print(longest_edge)
                hor = np.array([1,0])
                angle = 180/np.pi * acos((hor[0]*longest_edge[0] + hor[1]*longest_edge[1])/ (cv2.norm(hor)*cv2.norm(longest_edge)))
                print(angle)
            
                # if angle >= 160 and angle <= 180:
                #     self.label = 1
                #     # return 1 #right
                # else:
                #     self.label = -1
                #     # return -1 #left
        return 0
        
    def process_image(self, src):
        warped = None
        roi = src[:,:]
        h, w = roi.shape[:2]
        blue = self.blue_area(roi)

        res = 0
        contours = cv2.findContours(blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]

        if len(contour_sizes) > 0:
            biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]
            area = cv2.contourArea(biggest_contour)
            # print(area)
            if(area > 800):
                x,y,w,h = cv2.boundingRect(biggest_contour)
                if (abs(w-h) < (w+h)/12):
                    warped = roi[y:y+h,x:x+w]
            if warped is not None:
                gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
                if self.reprocess_tf:
                    gray[(gray >= 50) & (gray <= 120)] = 0
                thr = cv2.threshold(gray, self.thr_bin, 255, cv2.THRESH_BINARY)[1]
                thr = cv2.morphologyEx(thr, cv2.MORPH_OPEN, np.ones((2,2),np.uint8))
                res = self.direct_indentify(thr)
                if self._debug:
                    cv2.imshow("gray", gray) 
                    cv2.imshow("tf", thr)            
                    cv2.waitKey(1) 
        return res