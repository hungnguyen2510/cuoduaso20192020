#! /usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String,Float32
from cv_bridge import CvBridge
from team419.msg import Line
import cv2
import numpy as np
import imutils
import time
from pylab import array,uint8 
from Preprocess import FuncsPreprocess
from config import Config


rospy.init_node('traffic_sign_node', anonymous=True)
bridge = CvBridge()

imageDepth = None
arrCenterPoint = []
arrLeftPoint = []
arrRightPoint = []
config = Config()
TEAM = config.get_team()

def image_callback_depth(msg):
  global imageDepth
  imageDepth = bridge.compressed_imgmsg_to_cv2(msg,'mono8')

def removeGround(img):
  res = np.zeros(shape=(img.shape))
  temp = np.array(img)
  for i in range(img.shape[0]-1,0,-1):
    img[i,:] = img[i,:] - img[i-8,:]
  res[(img <= 10)] = 255
  res[temp > 125] = 0
  res[:,0:50] = 0
  res[:,270:] = 0 
  return res

def lane_point(ros_data):
  global arrCenterPoint,arrLeftPoint,arrRightPoint
  arrCenterPoint = list(ros_data.mid)
  arrLeftPoint = list(ros_data.left)
  arrRightPoint = list(ros_data.right)

def Average(lst): 
    return sum(lst) / len(lst) 

rospy.Subscriber('/' + TEAM + '/camera/depth/compressed',CompressedImage,image_callback_depth)
rospy.Subscriber('/'+ TEAM +'/lane_point',Line,lane_point)
kernel = np.ones((11, 11), np.uint8)
publish_obstacle = rospy.Publisher('/' + TEAM + '/obstacle', String, queue_size=1)

y = 0
x = 0
pre = FuncsPreprocess()
while not rospy.is_shutdown():
  if imageDepth is not None:
    frameDepth = imageDepth.copy()
    h, w = frameDepth.shape[:2]
    cropImage = frameDepth[95:y+h,0:320]
    cropImage = cropImage[0:30,0:320]
    remove = removeGround(cropImage)
    copyRemove = remove.copy()
    # copyRemove = cv2.cvtColor(copyRemove,cv2.COLOR_GRAY2RGB)
    # cv2.imshow('remove',remove)
    remove = 255 * remove # Now scale by 255
    remove = remove.astype(np.uint8)

    _, thresh = cv2.threshold(np.uint8(remove * 255), 30, 255, cv2.THRESH_BINARY)
    countours = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    countours = imutils.grab_contours(countours)
    contour_sizes = [(cv2.contourArea(contour), contour) for contour in countours]

    # depthRGB = cv2.cvtColor(frameDepth,cv2.COLOR_GRAY2RGB)
    # closing = cv2.morphologyEx(remove, cv2.MORPH_OPEN, kernel)
    
    ob_detect = 'None'
    if len(contour_sizes) > 0:
      bigest_area_cnt = max(contour_sizes, key=lambda x: x[0])[1]   
      area = cv2.contourArea(bigest_area_cnt) 
        
      if(area > config.get_limit_area_ob() and area < 1500):
        # print(area)    
        # cv2.drawContours(deptRGB,[c],0,(255,255,0),1)
        x,y,w,h = cv2.boundingRect(bigest_area_cnt)
        if (x > 70) and (x < 200):     
          M = cv2.moments(bigest_area_cnt)
          if(M['m00'] != 0):
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
          if(cx > 70) and (cx<150):
            # cv2.drawContours(copyRemove,[bigest_area_cnt],0,(255,255,0),1)
            # cv2.circle(copyRemove,(cx,cy),1,(255,255,0),2)
            ob_detect = 'objectleft'
            rospy.sleep(0.5)
          if(cx>155) and (cx < 200):
            # cv2.drawContours(copyRemove,[bigest_area_cnt],0,(0,255,255),1)
            # cv2.circle(copyRemove,(cx,cy),1,(255,255,0),2)
            ob_detect = 'objectright'
            rospy.sleep(0.5)
    publish_obstacle.publish(ob_detect)
    if(config.get_debug() == True):
      cv2.imshow('obstacle',copyRemove)
  cv2.waitKey(1)
