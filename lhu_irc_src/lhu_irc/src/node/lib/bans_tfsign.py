#!/usr/bin/env python

from cv_bridge import CvBridge
import cv2
import numpy as np
import imutils

class BansTFSign():
    def __init__(self):
        self.debug = 1
        self.kernel = np.ones((3,3),np.uint8)
    
    
    def detect_red_sign(self,frameRGB,frameDepth):
        res = None
        cropFrameRGB = frameRGB[0:160,:]
        # cropFrameDepth = frameDepth[0:160,:]
        
        hsvImage = cv2.cvtColor(cropFrameRGB,cv2.COLOR_RGB2HSV)
        low_val = np.array([117,94,36])
        high_val = np.array([179,255,255])
        mask = cv2.inRange(hsvImage,low_val,high_val)
        opening = cv2.morphologyEx(mask,cv2.MORPH_OPEN,self.kernel)
        dilete = cv2.dilate(opening,self.kernel,iterations = 3)
        circles = cv2.HoughCircles(dilete, cv2.HOUGH_GRADIENT, 0.5, dilete.shape[0]/64, param1=400, param2=10, minRadius=20, maxRadius=30)
        if circles is not None:
            res = True
            if self.debug:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    # Draw outer circle
                    cv2.circle(cropFrameRGB, (i[0], i[1]), i[2], (0, 255, 0), 2)
                    # Draw inner circle
                    cv2.circle(cropFrameRGB, (i[0], i[1]), 2, (0, 0, 255), 3)
        if self.debug:
            cv2.imshow('mask',dilete)
            cv2.imshow('cropFrameRGB',cropFrameRGB)
            cv2.waitKey(1)
        return res