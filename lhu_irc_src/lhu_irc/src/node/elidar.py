#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Int32MultiArray
import numpy as np
import cv2
import time
import collections
import imutils

#denoise configsrc

_debug = 1

centimet = lambda m: int(m*100)
points = 160
angle = 80 #angle
lidar_data = [0] * points

rangeDetect = 3 #pham vi quet

d = 5 #so lop khu nhieu
arr = collections.deque(maxlen=d)
isArr = False

#object detect
objSize = 60
dilate_pts = 8
vus = 30 #bo khoang gan nhat

#draw parabola
parabola = np.array([[82, 0], [82, 1], [82, 2], [82, 3], [82, 4], [82, 5], [82, 6], [82, 7], [82, 8], [82, 9], [82, 10], [82, 11], [82, 12], [82, 13], [82, 14], [82, 15], [82, 16], [82, 17], [82, 18], [82, 19], [82, 20], [82, 21], [83, 22], [83, 23], [83, 24], [83, 25], [83, 26], [83, 27], [83, 28], [83, 29], [83, 30], [83, 31], [83, 32], [83, 33], [83, 34], [83, 35], [83, 36], [83, 37], [83, 38], [83, 39], [83, 40], [83, 41], [83, 42], [83, 43], [83, 44], [83, 45], [83, 46], [83, 47], [83, 48], [83, 49], [83, 50], [83, 51], [83, 52], [83, 53], [83, 54], [83, 55], [84, 56], [84, 57], [84, 58], [84, 59], [84, 60], [84, 61], [84, 62], [84, 63], [84, 64], [84, 65], [84, 66], [84, 67], [84, 68], [84, 69], [84, 70], [84, 71], [84, 72], [84, 73], [84, 74], [84, 75], [84, 76], [84, 77], [84, 78], [84, 79], [84, 80], [84, 81], [84, 82], [84, 83], [84, 84], [84, 85], [84, 86], [84, 87], [84, 88], [84, 89], [84, 90], [84, 91], [84, 92], [84, 93], [85, 94], [85, 95], [85, 96], [85, 97], [85, 98], [85, 99], [85, 100], [85, 101], [85, 102], [85, 103], [85, 104], [85, 105], [85, 106], [85, 107], [85, 108], [85, 109], [85, 110], [85, 111], [85, 112], [85, 113], [85, 114], [85, 115], [85, 116], [85, 117], [85, 118], [85, 119], [85, 120], [85, 121], [85, 122], [85, 123], [85, 124], [85, 125], [85, 126], [85, 127], [85, 128], [85, 129], [86, 130], [86, 131], [86, 132], [86, 133], [86, 134], [86, 135], [86, 136], [86, 137], [86, 138], [86, 139], [86, 140], [86, 141], [86, 142], [86, 143], [86, 144], [86, 145], [86, 146], [86, 147], [86, 148], [87, 149], [87, 150], [87, 151], [87, 152], [87, 153], [87, 154], [87, 155], [87, 156], [87, 157], [87, 158], [88, 159], [88, 160], [88, 161], [88, 162], [88, 163], [88, 164], [88, 165], [88, 166], [88, 167], [88, 168], [88, 169], [88, 170], [88, 171], [89, 172], [89, 173], [89, 174], [89, 175], [89, 176], [89, 177], [89, 178], [89, 179], [89, 180], [89, 181], [89, 182], [89, 183], [89, 184], [89, 185], [89, 186], [90, 187], [90, 188], [90, 189], [90, 190], [90, 191], [90, 192], [90, 193], [90, 194], [91, 195], [91, 196], [91, 197], [91, 198], [91, 199], [92, 200], [92, 201], [92, 202], [92, 203], [92, 204], [92, 205], [92, 206], [93, 207], [93, 208], [93, 209], [93, 210], [93, 211], [93, 212], [93, 213], [93, 214], [94, 215], [94, 216], [94, 217], [94, 218], [94, 219], [95, 220], [95, 221], [95, 222], [96, 223], [96, 224], [96, 225], [96, 226], [97, 227], [97, 228], [97, 229], [97, 230], [97, 231], [98, 232], [98, 233], [98, 234], [98, 235], [99, 236], [99, 237], [100, 238], [100, 239], [101, 240], [101, 241], [101, 242], [102, 243], [102, 244], [103, 245], [103, 246], [104, 247], [104, 248], [105, 249], [105, 250], [105, 251], [106, 252], [106, 253], [107, 254], [108, 255], [109, 256], [110, 257], [110, 258], [111, 259], [112, 259], [113, 260], [114, 261], [115, 262], [116, 263], [117, 264], [118, 265], [119, 266], [120, 266], [121, 266], [122, 267], [123, 268], [124, 269], [125, 269], [126, 269], [127, 270], [128, 270], [129, 270], [130, 271], [131, 271], [132, 271], [133, 272], [134, 272], [135, 273], [136, 273], [137, 273], [138, 273], [139, 273], [140, 274], [141, 274], [142, 274], [143, 275], [144, 275], [145, 275], [146, 275], [147, 275], [148, 276], [149, 276], [150, 276], [151, 276], [152, 276], [153, 276], [154, 276], [155, 277], [156, 277], [157, 277], [158, 277], [159, 277]],
            dtype='int')

def object_finding(src,arr):

    img = src.copy()
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    rightObject = []
    leftObject = []
    centerObject = []

    contours = cv2.findContours(src, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    contours = imutils.grab_contours(contours)

    leftState, centerState, rightState = 0, 0, 0

    if len(contours) > 0:
        for cnt in contours:

            x,y,w,h = cv2.boundingRect(cnt)
            x1, x2, y1, y2 = x, x+w, y, y+h

            col = parabola[y+(h//2)][0]

            d = np.sqrt((x2-x1)**2 + (y2-y1)**2)

            d = round(d, 2)

            cv2.putText(img,'{}'.format(d),(x1,y1-10),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,0,255),1)

            if d > 25:

                #left area
                if x1 < points-col and x2 < points - col and y2 < centimet(rangeDetect)-vus:
                    leftObject.append(cnt)


                if x1 < points-col and x2 > points-col and y2 < centimet(rangeDetect)-vus:
                    if ((points - col)-x1) < (x2-(points-col)):
                        centerObject.append(cnt)
                    else:
                        leftObject.append(cnt)

                # #center area
                if (col > x1 > points-col and col > x2 > points-col) and y2 < centimet(rangeDetect)-vus:
                    centerObject.append(cnt)

                #right area
                if x1 > col and x2 > col and y2 < centimet(rangeDetect)-vus:
                    rightObject.append(cnt)

                if x1 < col and x2 > col and y2 < centimet(rangeDetect)-vus:
                    if (x2 - col) < (col - x1):
                        centerObject.append(cnt)
                    else:
                        rightObject.append(cnt)
            if _debug:
                cv2.drawContours(img, [cnt], -1, (0,255,0),-1)
    if _debug:
        cv2.imshow('rgb',img)    

    return leftObject, centerObject, rightObject


def histImg(arr):
    x, y = [points,centimet(rangeDetect)]
    imgHist = np.zeros((y,x),dtype= np.uint8)
    for i in range(x):
        if arr[i] < centimet(rangeDetect):
            cv2.circle(imgHist,(i, y - arr[i]),dilate_pts,255,-1)

    return imgHist

def denoiseLidar():
    global arr
    arr.append(lidar_data)
    new = [0] * points
    for i in range(points):
        tmp_count = 0
        tmp = None
        for j in range(1,len(arr)):
            if arr[j][i] >= centimet(rangeDetect):
                tmp_count += 1
            if arr[j][i] < centimet(rangeDetect):
                if tmp is None:
                    new[i] = int(arr[j][i])
                    tmp = new[i]

                else:
                    new[i] = int(0.5 * new[i] + 0.5 * arr[j][i])
                tmp_count = 0
                
        if tmp_count == len(arr)-1:
            new[i] = centimet(rangeDetect)

    return new

def scan_callback(msg):
    global lidar_data, isArr
    leftArea = list(msg.ranges[angle:0:-1])
    rightArea = list(msg.ranges[359:359-angle:-1])
    concat_data = np.hstack([leftArea,rightArea])
    lidar_data = [centimet(rangeDetect) if (str(i) == 'inf' or i > rangeDetect) else i * 100 for i in concat_data]
    isArr = True

rospy.init_node('lidar_node', anonymous=True)
rospy.Subscriber('/scan', LaserScan, scan_callback)

while not rospy.is_shutdown():

    if isArr:
        
        ref_lidar_data = denoiseLidar()

        out_src = histImg(ref_lidar_data)
        
        leftObject, centerObject, rightObject = object_finding(out_src,ref_lidar_data)

        print(len(leftObject),len(centerObject), len(rightObject))

        isArr = False

        if _debug:
            cv2.imshow('src',out_src)
            cv2.waitKey(1)
        
    time.sleep(0.01)