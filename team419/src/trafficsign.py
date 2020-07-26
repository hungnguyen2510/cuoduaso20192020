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
os.environ["CUDA_VISIBLE_DEVICES"] = ""

from keras.models import load_model
import util
import numpy as np
from config import Config

rospy.init_node("node_main",anonymous=True)
bridge =  CvBridge()
image = None
debug = False
config = Config()
def image_callback(msg):
    global image
    image = bridge.compressed_imgmsg_to_cv2(msg,'bgr8')

def nothing(x):
    pass

TEAM = config.get_team()
rospy.Subscriber('/' + TEAM + '/camera/rgb/compressed',CompressedImage,image_callback)
publish_traffic = rospy.Publisher('/' + TEAM + '/traffic', String, queue_size=1)

modelPath =  os.path.join(os.path.dirname(os.path.realpath(__file__)), 'model', 'new_model_90-1.00.h5')
model = load_model(modelPath)

print('Traffic sign running!!!!')
while not rospy.is_shutdown():
  if image is not None:
    if debug == False:    
      frame = image.copy() 
      frame=frame[50:110,160:] 
      # cv2.imshow('a',frame)  
      hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

      #map 2
      # lower_range = np.array([105,103,40])
      # upper_range = np.array([115,135,80])

      # lower_range = np.array([70,60,60])
      # upper_range = np.array([115,150,115])

      lower_range = np.array([100,60,35])
      upper_range = np.array([115,150,115])


      # lower_range = np.array([100,50,0])
      # upper_range = np.array([120,255,140])

      mask = cv2.inRange(hsv, lower_range, upper_range)

      kernel = np.ones((3,3), np.uint8)
      # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
      mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
      mask = cv2.dilate(mask, kernel, iterations=1)

      # maskTwo = cv2.cvtColor(mask,cv2.COLOR_GRAY2RGB)

      # output = cv2.bitwise_and(image, image, mask = mask)

      contours = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
      # contours = imutils.grab_contours(contours)
      img =  None
      tf_sign = 'None'
      for c in contours:
        area = cv2.contourArea(c)
        # cv2.drawConatours()
        # if area > 150:
        if(area > 150 and area<600):
            x,y,w,h = cv2.boundingRect(c)
            if (abs(w-h) < (w+h)/10):
                img = frame[y:y+h,x:x+w]
      if img is not None:
        # cv2.imshow('imge',img)
        imagePredict = util.resizeImage(img)
        imagePredict = imagePredict / 255.0
        predicted = model.predict(np.array([imagePredict]))
        # print (predicted)
        if predicted[0][0] <= 0.2:
            # print('left')
            tf_sign = 'left'
            # rospy.sleep(1)
        if predicted[0][0] > 0.8 and predicted[0][0] <= 1.0:
            # print('right'
            tf_sign = 'right'
      publish_traffic.publish(tf_sign)
      if(config.get_debug() == True):
          cv2.imshow('output', mask)
      # cv2.imshow('hsv', hsv)
    else:
      debugImage = image.copy()
      hsv_img = cv2.cvtColor(debugImage, cv2.COLOR_BGR2HSV)
      cv2.namedWindow('HSV')
      hl = 'hue low'
      hh = 'hue high'
      sh = 'saturation high'
      sl = 'saturation low'
      vh = 'value high'
      vl = 'value low'
      wnd = 'HSV'
      cv2.createTrackbar(hl, wnd, 0,179, nothing)
      cv2.createTrackbar(hh, wnd, 0,179, nothing)
      cv2.createTrackbar(sl, wnd, 0,255, nothing)
      cv2.createTrackbar(sh, wnd, 0,255, nothing)
      cv2.createTrackbar(vl, wnd, 0,255, nothing)
      cv2.createTrackbar(vh, wnd, 0,255, nothing)
      
      hul= cv2.getTrackbarPos(hl,wnd)
      huh= cv2.getTrackbarPos(hh,wnd)
      sal= cv2.getTrackbarPos(sl,wnd)
      sah= cv2.getTrackbarPos(sh,wnd)
      val= cv2.getTrackbarPos(vl,wnd)
      vah= cv2.getTrackbarPos(vh,wnd)

    # print(hul)

      hsvl = np.array([hul, sal, val])
      hsvh = np.array([huh, sah, vah])
      mask = cv2.inRange(hsv_img, hsvl, hsvh)       
  key = cv2.waitKey(1)
  if key == ord('b'):
    cv2.destroyWindow('output')
    cv2.destroyWindow('hsv')
    debug = True
  if key == ord('c'):
    cv2.destroyAllWindows()
    debug = False
