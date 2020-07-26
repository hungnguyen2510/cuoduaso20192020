#!/usr/bin/env python
import cv2
import numpy as np


def extract_horizontal_line(img, kenel=(6, 50)):
    
    linek = np.zeros(kenel, dtype=np.uint8)
    linek[kenel[0] // 2 - 1, ...] = 1

    horizontal_line = cv2.morphologyEx(img, cv2.MORPH_OPEN, linek, iterations=1)
   

    return horizontal_line


def identify_area_parking(img):
    debug = 1
    img = img[140:190, :]
    lower_white = np.array([187, 193, 187], np.uint8)
    upper_white = np.array([255, 255, 255], np.uint8)
    white_lane = cv2.inRange(img, lower_white, upper_white)
    white_lane = extract_horizontal_line(white_lane, kenel=(8, 80))
    if debug:
        cv2.imshow('debug-parking',white_lane)
        cv2.waitKey(1)
    cnts = cv2.findContours(white_lane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    k = 0
    for cnt in cnts:
        area = cv2.contourArea(cnt)
        k += area

        if k > 800:
            return 1
        else:
            return -1

