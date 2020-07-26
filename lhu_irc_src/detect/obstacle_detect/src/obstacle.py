#!/usr/bin/env python

from cv_bridge import CvBridge
import cv2
import numpy as np
import imutils

class Obstacle():
	def __init__(self):
		self.areaOb = 1000
		self.kernel = np.ones((5,5),np.uint8)
		self.debug = 1

	def find_closest_obstacle(self,rgb_image,depth_image):
		ob_result,mask_depth = -999, None
		h, w = rgb_image.shape[:2]
		cropImage = rgb_image[0:160,:]
		cropImageDepth = depth_image[0:160,:]

		hsvImage = cv2.cvtColor(cropImage,cv2.COLOR_RGB2HSV)
		low_value =  np.array([117,94,36])
		high_value =  np.array([179,255,255])
		mask = cv2.inRange(hsvImage, low_value, high_value)

		opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
		dilete = cv2.dilate(opening,self.kernel,iterations = 2)
		contours = cv2.findContours(dilete, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
		contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]
		
		if len(contour_sizes) > 0:
			biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]
			area = cv2.contourArea(biggest_contour)
			if area > self.areaOb:
				cv2.drawContours(rgb_image,[biggest_contour],0,(255,0,255),2)
				M = cv2.moments(biggest_contour)
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])

				cv2.circle(rgb_image,(cX,cY),1,(255,0,230),3)
				mask_depth = cv2.bitwise_and(cropImageDepth,cropImageDepth,mask=dilete)
				ob_result = mask_depth[cY,cX]
		if self.debug:
			cv2.imshow('rgb_image',rgb_image)
			cv2.imshow('dilete',dilete)
			if mask_depth is not None:
				cv2.imshow('mask_depth',mask_depth)
				# cv2.imshow('depth_frame',depth_frame)
			if mask_depth is None:
				cv2.destroyWindow('mask_depth')
			cv2.waitKey(1)
		return ob_result