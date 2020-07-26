#! /usr/bin/env python 
from __future__ import division
import rospy 
from std_msgs.msg import Float32, String,Bool
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from Preprocess import FuncsPreprocess
from team419.msg import Line
from config import Config
import cv2
import math
import imutils
import numpy as np
import os
import time
from pylab import array,uint8 
# from simple_pid import PID

pre = FuncsPreprocess()
config = Config()
ob_detected = ''
traffic_sign = ''

rospy.init_node("node_main",anonymous=True)

bridge =  CvBridge()

image = None
imageDepth = None
debug = False
arrCenterPoint,arrCenterPointLeft,arrCenterPointRight = [], [], []
arrLeftPoint = []
arrRightPoint = []

center = 160
detect_snow = False
arrTamSnow = []
lane_cong_trai = False
lane_cong_phai = False
count = 0

def lane_point(ros_data):
    global arrCenterPoint,arrLeftPoint,arrRightPoint,arrCenterPointLeft,arrCenterPointRight
    arrCenterPoint = list(ros_data.mid)
    arrLeftPoint = list(ros_data.left)
    arrRightPoint = list(ros_data.right)
    if list(ros_data.left) != []:
      arrCenterPointLeft = list(ros_data.goleft)
    if list(ros_data.right) != []:
      arrCenterPointRight = list(ros_data.goright)
    # print(ros_data.data)

def callback_tf_sign(ros_data):
    global traffic_sign
    # print("traffic_sign: ", ros_data.data)
    traffic_sign = ros_data.data

def callback_ob_detected(ros_data):
    global ob_detected
    # print("ob_detected: ", ros_data.data)
    ob_detected = ros_data.data

def snow_detect(ros_data):
    global detect_snow
    detect_snow = ros_data.data


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
    for i in range(len(arr)-4,0,-1):
        FivePointFirst.append(arr[i])
        # print(FivePointFirst)
    for j in range(5,len(arr)-1,1):
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
    
def FindCenterPoint(arrCenter):
    if arrCenter != []:
        center = arrCenter[0]
    else:
        center = 160          
    for i in range(1,len(arrCenter)):
        center = 0.7*center + 0.3*arrCenter[i]
    return center

resetTime = time.time()
def delay(second):
  if(time.time() - resetTime) > second:
      return False
  return True

def resetT():
  global resetTime
  resetTime = time.time()

######################################################################
def CheckThangHang():
  offset = 10
  offsetLen = 5
  if(len(arrCenterPoint) > offsetLen and len(arrLeftPoint) > offsetLen and len(arrRightPoint) > offsetLen):
    for i in range(offsetLen-1):
      if abs(arrCenterPoint[i] - arrCenterPoint[i+1]) > offset or abs(arrLeftPoint[i] - arrLeftPoint[i+1]) > offset or abs(arrRightPoint[i] - arrRightPoint[i+1]) > offset:
        return False
    return True
  return False

def CheckTraiThangHang():
  offsetLen = 4
  offset = 50
  if len(FivePointLeft) > offsetLen:
    for i in range(len(arrLeftPoint)-1):
      if abs(arrLeftPoint[i] - arrLeftPoint[i+1]) > offset:
        return False
    return True
  return False

def CheckPhaiThangHang():
  offsetLen = 4
  offset = 50
  if len(FivePointRight) > offsetLen:
    for i in range(len(arrRightPoint)-1):
      if abs(arrRightPoint[i] - arrRightPoint[i+1]) > offset:
        return False
    return True
  return False

def CheckDuongCong():
  if arrCenterPoint != []:
    avgDuongCong = Average(arrCenterPoint)
    if ob_detected == 'None' and traffic_sign == 'None':
      if(avgDuongCong > 0 and  avgDuongCong < 145):
        GoLeft(35)
        # print('duong cong trai')
      if(avgDuongCong > 180):
        GoRight(35)
        # print('duong cong phai')
#####################################################################

##############################################################

def GoLeft(speed):
  # print('go left')
  centerLeft = FindCenterPoint(arrCenterPointLeft)
  angle = int(((centerLeft - 160)/160) * 60) 
  pub_angle.publish(angle)
  pub_speed.publish(speed) 

def GoRight(speed):
  # print('go right')
  centerRight = FindCenterPoint(arrCenterPointRight)
  angle = int(((centerRight - 160)/160) * 60) 
  pub_angle.publish(angle)
  pub_speed.publish(speed)

def GoStraight(speed):
  global center
  center = FindCenterPoint(arrCenterPoint)
  angle = int(((center - 160)/160) * 60) 
  pub_angle.publish(angle)
  pub_speed.publish(speed) 
  pub_angle_camera.publish(0)   

def SnowLeft(ispeed,iangle,timesleep):
  resetT()
  while delay(timesleep):
    pub_speed.publish(ispeed)                     
    pub_angle.publish(iangle)
    time.sleep(0.01)
  lane_cong_trai = False

def SnowRight(ispeed,iangle,timesleep):
  resetT()
  while delay(timesleep):
    pub_speed.publish(ispeed)                     
    pub_angle.publish(iangle)
    time.sleep(0.01)
  lane_cong_phai = False
###################################################################

def ProcessTFLeft(speed,angleInput,time_input,defi):
  global center
  while(len(arrCenterPoint)>defi):
    angle = int(((center - 160)/160) * 60)
    pub_angle.publish(angle)
    pub_speed.publish(speed-5)
    time.sleep(0.01) 
  resetT()
  while not CheckThangHang() or delay(time_input):
    pub_angle.publish(angleInput)
    pub_speed.publish(speed+5)
    time.sleep(0.01)

def ProcessTFRight(speed,angleInput,time_input,defi):
  global center
  while(len(arrCenterPoint)>defi):
      angle = int(((center - 160)/160) * 60)
      pub_angle.publish(angle)
      pub_speed.publish(speed-5)
      time.sleep(0.01) 
  resetT()
  while not CheckThangHang() or delay(time_input):
    pub_angle.publish(angleInput)
    pub_speed.publish(speed+5)
    time.sleep(0.01)

def ProcessObLeft(speed,time_input):
  resetT()
  while delay(time_input):
    GoRight(speed)
    time.sleep(0.01)


def ProcessObRight(speed,time_input):
  resetT()
  while delay(time_input):
    GoLeft(speed)
    time.sleep(0.01)

###################################################################

angle = 0.0
angle_camera = 0.0
TEAM = config.get_team()

pub_speed = rospy.Publisher('/'+ TEAM +'/set_speed',Float32,queue_size=1)
pub_angle = rospy.Publisher('/'+ TEAM +'/set_angle',Float32,queue_size=1)
pub_angle_camera = rospy.Publisher('/'+ TEAM +'/set_camera_angle',Float32,queue_size=1)

rospy.Subscriber('/'+ TEAM +'/lane_point',Line,lane_point)
rospy.Subscriber('/'+ TEAM +'/snow',Bool,snow_detect)
rospy.Subscriber('/'+ TEAM +'/traffic',String,callback_tf_sign)
rospy.Subscriber('/'+ TEAM +'/obstacle', String, callback_ob_detected)

                                                        

print('Node main running.....!')
while not rospy.is_shutdown():
  if(traffic_sign == 'left'): #bien bao trai 
    # print('traffic-left')
    ProcessTFLeft(config.get_speed_process_tf(),config.get_angle_tf_left(),config.get_time_delay_tf_left(),3) #speed,angle,time_input,defi
  if(traffic_sign == 'right'): #bien bao phai 
    # print('traffic-right')
    ProcessTFRight(config.get_speed_process_tf(),config.get_angle_tl_right(),config.get_time_delay_tf_right(),3) #speed,angle,time_input,defi

  if(ob_detected == 'objectright'):
    # print('ob-right')
    ProcessObRight(config.get_speed_process_ob(),config.get_time_delay_ob_right()) #speed,time_input
  if(ob_detected == 'objectleft'):
    # print('ob-left')
    ProcessObLeft(config.get_speed_process_ob(),config.get_time_delay_ob_left()) #speed,time_input

  if(detect_snow == False):
    GoStraight(config.get_speed_go_straight()) #speed
    CheckDuongCong()
    if(arrCenterPoint != []):
      arrTamSnow = arrCenterPoint
  else:
    DetectLaneNoneLine(arrTamSnow)
    if (lane_cong_trai == True):                                      
      SnowLeft(config.get_speed_go_snow(),config.get_angle_snow_left(),config.get_time_delay_snow_left()) #speed,angle,time_input
    if (lane_cong_phai == True):                                      
      SnowRight(config.get_speed_go_snow(),config.get_angle_snow_right(),config.get_time_delay_snow_right()) #speed,angle,time_input
  time.sleep(0.01)
  key = cv2.waitKey(1)
