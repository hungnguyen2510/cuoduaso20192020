import numpy as np
import cv2
import imutils

class FuncsPreprocess:
  def __init__(self):
    self.w = 320
    self.h = 240
    self.src_birdview = np.array([[0, self.h/2], [self.w, self.h/2], [self.w, 180], [0, 180]], np.float32)
    self.dst_birdview = np.array([[0, 0], [self.w, 0], [self.w-80, self.h], [80, self.h]], np.float32)
    self.M_birdview = cv2.getPerspectiveTransform(self.src_birdview, self.dst_birdview)
    self.M_reverse_birdview = cv2.getPerspectiveTransform(self.dst_birdview, self.src_birdview)
    self.Canny_thresh = [100, 200]
    self.Threshold = [170, 255]
    self.Gaussian_Kernel = (5,5)
    self.Erode_Kernel = (1,3)
    self.Dilate_Kernel = (3,1)


  def FuncCanny(self, img):
    return cv2.Canny(img, self.Canny_thresh[0], self.Canny_thresh[1], apertureSize=3)

  def BGRtoGray(self, img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

  def FuncThresshold(self, img):
    return cv2.threshold(self.BGRtoGray(img), self.Threshold[0], self.Threshold[1], cv2.THRESH_BINARY)[1]

  def FuncBlur(self, img):
    return cv2.GaussianBlur(img, self.Gaussian_Kernel, 0)
  
  def FuncErode(self,img):
    return cv2.erode(img,cv2.getStructuringElement(cv2.MORPH_CROSS,self.Erode_Kernel),iterations=1)

  def FuncDilate(self,img):
    return cv2.dilate(img,cv2.getStructuringElement(cv2.MORPH_CROSS,self.Dilate_Kernel),iterations=1)

  def FuncBirdView(self, img):
    return cv2.warpPerspective(img.copy(), self.M_birdview, (320, 240))

  def FuncTranformBirdView(self, img):
    return cv2.warpPerspective(img.copy(), self.M_reverse_birdview, (320, 240))

  def reverseTranformBirdView(self, arrP):
    return cv2.perspectiveTransform(arrP, self.M_reverse_birdview)

