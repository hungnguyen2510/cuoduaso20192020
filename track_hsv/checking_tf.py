import cv2
import imutils
import numpy as np

import os
from keras.models import load_model

class Traffic():
    def __init__(self):
        # self.lower_range = np.array([43,100,121])
        # self.upper_range = np.array([162,255,255])

        self.lower_range = np.array([94,94,71])
        self.upper_range = np.array([162,255,255])
        self.kernel = np.ones((3,3), np.uint8)
        self.modelPath =  os.path.join(os.path.dirname(os.path.realpath(__file__)), 'model', 'straight_64-1.00.h5')
        self.model = load_model(self.modelPath)

        self.INPUT_SIZE=(24,24,3)

    def resizeImage(self,img):
        return cv2.resize(img, (self.INPUT_SIZE[0], self.INPUT_SIZE[1]), interpolation = cv2.INTER_AREA)

    def detect_traffic_sign(self,frame):
        img,tf_sign = None, None
        frame=frame[60:140,:]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # lower_range = np.array([54,81,53])
        # upper_range = np.array([162,255,255])
        
        mask = cv2.inRange(hsv, self.lower_range, self.upper_range)      
        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        mask = cv2.dilate(mask, self.kernel, iterations=1)
        # contours = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]

        if len(contour_sizes) > 0:
            biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]
            area = cv2.contourArea(biggest_contour)
            # print(area)
            if(area > 800):
                x,y,w,h = cv2.boundingRect(biggest_contour)
                if (abs(w-h) < (w+h)/12):
                    img = frame[y:y+h,x:x+w]
                    cv2.imshow('img',img)
        if img is not None:
            imagePredict = self.util.resizeImage(img)
            imagePredict = imagePredict / 255.0
            predicted = self.model.predict(np.array([imagePredict]))
            # print(predicted[0][0])
            if predicted[0][0] <= 0.2:
                tf_sign = 'notstraight'
            if predicted[0][0] > 0.8 and predicted[0][0] <= 1.0:
                tf_sign = 'straight'
            # cv2.imshow('imge',img)
            
            # cv2.imshow('frame',frame)
            # cv2.imshow('output', mask)
            # cv2.waitKey(1)
        return tf_sign, mask

if __name__ == '__main__':
    vidcap = cv2.VideoCapture("/home/p2h/Desktop/track_hsv/data/1902_151540.avi")
    print vidcap.read()
    success,image = vidcap.read()
    count = 0
    success = True
    while success:
        success,image = vidcap.read()
        cv2.imshow('image',image)
        cv2.waitKey(0)