#! /usr/bin/env python 
from __future__ import division
import rospy 
from std_msgs.msg import Float32, String,Bool
from team419.msg import Line
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from Preprocess import FuncsPreprocess
import cv2
import math
import imutils
import numpy as np
import os
import time
from pylab import array,uint8 
from config import Config

rospy.init_node("node_main",anonymous=True)
bridge =  CvBridge()
config = Config()

pre = FuncsPreprocess()
ob_detected = False
traffic_sign = ''

image = None
imageDepth = None
debug = False

detect_snow = False
lane_cong_trai = False
lane_cong_phai = False
xPoint = 160
H_LANE = 240//3

savePointLeft, savePointRight = [], []

def image_callback(msg):
  global image
  image = bridge.compressed_imgmsg_to_cv2(msg,'bgr8')

def image_callback_depth(msg):
  global imageDepth
  imageDepth = bridge.compressed_imgmsg_to_cv2(msg,'mono8')

############################################################################################################################
def XuongCa(imgRgb, img):
  global disLane,savePointLeft, savePointRight
  ycenterArr = []
  minLeft, minRight = None, None
  cntLeft, cntRight = {}, {}
  mid = 160
  finalLeft, finalRight = None, None
  result = np.zeros_like(img)
  resultL, resultR = np.zeros_like(img), np.zeros_like(img)
  hshape,wshape = imgRgb.shape[:2]
  centertmp = int(wshape/2)
  arrayCenterLeft, arrayCenterRight,arrayCenter = [], [], []
  findBoxL, findBoxR = None, None
  disLane = 0
  edge1, edge2,longest_dis,shortest_dis = 0,0,0,0
  contours = cv2.findContours(img.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
  contours = imutils.grab_contours(contours) 
  
  for cnt in contours:
    minRect = cv2.minAreaRect(cnt)
    (x, y), (w, h), theta = minRect # (x, y) is center point of cnt
    area = cv2.contourArea(cnt)
    box = cv2.boxPoints(minRect)
    box = np.int0(box)
    if(w > h):
      w, h = h, w
    if(x < mid and h >= H_LANE): # filter with position and height
      if(cntLeft.get(x) is None):
        cntLeft.update({x: box})

    if(x >= mid and h >= H_LANE): # filter with position and height
      if(cntRight.get(x) is None):
        cntRight.update({x: box})
  finalLeft =  max(cntLeft.keys()) if len(cntLeft.keys()) > 0 else None
  finalRight = min(cntRight.keys()) if len(cntRight.keys()) > 0  else None

  if(finalLeft is not None):
    findBoxL = cntLeft.get(finalLeft)
    maskL = np.zeros_like(img)
    maskL = cv2.drawContours(maskL,[findBoxL],0,(255,255,255),-1)
    cv2.drawContours(imgRgb,[findBoxL],0,(0,255,255),3)
    resultL = cv2.bitwise_and(img, maskL)
    # cv2.imshow("maskLeft", maskL)
    # cv2.imshow("result1", result)

  if(finalRight is not None):
    findBoxR = cntRight.get(finalRight)
    maskR = np.zeros_like(img)
    maskR = cv2.drawContours(maskR,[findBoxR],0,(255,255,255),-1)
    cv2.drawContours(imgRgb,[findBoxR],0,(0,255,255),3)
    resultR = cv2.bitwise_and(img, maskR)
    # cv2.imshow("maskRight", maskR)
    # cv2.imshow("result2", result)
  result = cv2.bitwise_or(resultL, resultR)
  
  for y in range(hshape - 1, -1, -24):  
    ycenterArr.append(y)
    if(findBoxL is not None):
      for xleft in range(centertmp, -1, -1):
        if(result[y][xleft] != 0):
          cv2.circle(imgRgb,(xleft,y),1,(0,0,255),10)
          cv2.line(imgRgb,(centertmp,y),(xleft,y),(0,255,255),1)
          arrayCenterLeft.append(xleft)
          break 
    if(findBoxR is not None):
      for xright in range(centertmp,wshape-1,1):
        if(result[y][xright] != 0):
          cv2.circle(imgRgb,(xright,y),1,(0,0,255),10)
          cv2.line(imgRgb,(centertmp,y),(xright,y),(0,255,255),1)
          arrayCenterRight.append(xright)
          break
    if(arrayCenterLeft != []):
      savePointLeft = arrayCenterLeft
    if(arrayCenterRight != []):
      savePointRight = arrayCenterRight 
    if(len(arrayCenterLeft) < len(arrayCenterRight)):
      tmp = len(arrayCenterRight) - len(arrayCenterLeft)
      preValLeft = savePointLeft[len(savePointLeft) - 2] if len(savePointLeft) > 0 else 110
      for i in range(tmp):             
        arrayCenterLeft.append(preValLeft)
    else:
      tmp = len(arrayCenterLeft) - len(arrayCenterRight)          
      preValRight = savePointRight[len(savePointRight) - 2] if len(savePointRight) > 0 else 210
      for j in range(tmp):
        arrayCenterRight.append(preValRight)
    if findBoxR is not None and findBoxL is not None:
      for i in range(len(arrayCenterLeft)):
        center = int((arrayCenterLeft[i] + arrayCenterRight[i])/2)
        arrayCenter.append(center)
        cv2.circle(birdviewRGB,(center,ycenterArr[i]),1,(255,255,100),10)
        disLane = arrayCenterRight[i] - arrayCenterLeft[i]
  pos_ob = 0      
  
  if(disLane < config.get_limit_dis_lane()):
    pos_ob = int(disLane/6)
  else:
    pos_ob = config.get_pos_ob()
  # print(disLane,pos_ob)  
  # cv2.imshow('imgRgb', imgRgb)
  return imgRgb,arrayCenter,arrayCenterLeft,arrayCenterRight,ycenterArr,pos_ob

def DetectSnow(img):
  global detect_snow
  w, h = img.shape[:2]
  x = 0
  y = 0
  snow = img[100:y+int(h/2),x:x+w]
  thress = cv2.cvtColor(snow, cv2.COLOR_BGR2GRAY)
  thress = cv2.threshold(thress, 170, 200, cv2.THRESH_BINARY)[1]
  count = cv2.countNonZero(thress)
  # print(count)
  if(count > 8000):    
      detect_snow = True        
  else:
      detect_snow = False
    
    # cv2.imshow('thress',thress)

def Average(lst): 
    return sum(lst) / len(lst) 

def DetectLaneNoneLine(arr):
  global lane_cong_trai
  global lane_cong_phai
  FivePointFirst = []
  FivePointLast = []
  averageFirst = 0
  averageLast = 0
  # print(arr)
  for i in range(len(arr)-3,0,-1):
      FivePointFirst.append(arr[i])
      # print(FivePointFirst)
  for j in range(6,len(arr)-1,1):
      FivePointLast.append(arr[j])
      # print(FivePointLast)
  if(FivePointFirst != []) and (FivePointLast != []):
      averageFirst = Average(FivePointFirst)
      averageLast = Average(FivePointLast)    
  average = averageFirst - averageLast
  # print(average)
  if average < 0:
      # print('Snow-Right')
      lane_cong_phai = True
  else:
      # print('Snow-Left')
      lane_cong_trai = True



def FindArrLeft(img,arrL,ycenter, pos):
  # print('Go Left')
  centerLeftArr = []
  # if boxR is None and boxL is not None:
  for i in range(len(arrL)):
    tmpLeft = arrL[i] + pos
    # centerLeft = int((tmpLeft + arrR[i])/2)
    centerLeftArr.append(tmpLeft)
    cv2.circle(img,(tmpLeft,ycenter[i]),1,(255,0,100),10)
  return centerLeftArr

def FindArrRight(img,arrR,ycenter, pos):
  # print('Go Left')
  centerRightArr = []
  # if boxR is not None and boxL is None:
  for i in range(len(arrR)):
    tmpRight = arrR[i] - pos
    # centerRight = int((arrL[i] + tmpRight)/2)
    centerRightArr.append(tmpRight)
    cv2.circle(img,(tmpRight,ycenter[i]),1,(255,0,100),10)
  return centerRightArr
  

y = 0
x = 0

angle = 0.0
angle_camera = 0.0


TEAM = config.get_team()
pub_lanedetect = rospy.Publisher('/'+ TEAM +'/lane_point',Line,queue_size=1)
pub_snow = rospy.Publisher('/'+ TEAM +'/snow',Bool,queue_size=1)

rospy.Subscriber('/'+ TEAM +'/camera/rgb/compressed',CompressedImage,image_callback)
rospy.Subscriber('/' + TEAM + '/camera/depth/compressed',CompressedImage,image_callback_depth)

data = Line()

############################## MAIN #####################################################
print('Lane detect is running.....!')
while not rospy.is_shutdown():    
    if image is not None:     
        frame = image.copy()

        # Preprocess image
        blurred = pre.FuncBlur(frame)
        canny = pre.FuncCanny(blurred)
        
        # transform image
        birdviewRGB = pre.FuncBirdView(frame)
        birdview = pre.FuncBirdView(canny)

        # morph
        birdview = pre.FuncErode(birdview)
        birdview = pre.FuncDilate(birdview)
        
        #######################################################################################################
        imgRGB,arrayCenter,arrayLeft,arrayRight,yArrCenter,pos_ob = XuongCa(imgRgb=birdviewRGB, img=birdview)
        ArrPointLaneLeft = FindArrLeft(imgRGB,arrayLeft,yArrCenter,pos_ob)
        ArrPointLaneRight = FindArrRight(imgRGB,arrayRight,yArrCenter,pos_ob)
        ########################################################################################################


        ######################################################################################################
        DetectSnow(frame)
        pub_snow.publish(detect_snow)
        ######################################################################################################
        

        if(config.get_debug() == True):   
          cv2.imshow("image",image)
          cv2.imshow('canny',canny)
          cv2.imshow('birdview',birdview)
          cv2.imshow('imgRGB',imgRGB)
          cv2.waitKey(1) 
        #                                
        data.mid = arrayCenter
        data.left = arrayLeft
        data.right = arrayRight               
        if(ArrPointLaneLeft != []):
          data.goleft = ArrPointLaneLeft
        else:
          data.goleft = []
        if(ArrPointLaneRight != []):
          data.goright = ArrPointLaneRight
        else:
          data.goright = []
        pub_lanedetect.publish(data)
        cv2.waitKey(1)
