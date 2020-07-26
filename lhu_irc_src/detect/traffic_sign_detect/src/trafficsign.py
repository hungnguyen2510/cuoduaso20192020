#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32, Int32, Int32MultiArray, Bool
from cv_bridge import CvBridge
import cv2
import imutils
import numpy as np
import util
import os, time
import tensorflow
from tensorflow.keras.layers import Input,Conv2D,MaxPool2D,Dropout,Dense,Flatten,BatchNormalization,Activation
from tensorflow.keras.models import Model
from util import INPUT_SIZE

def build_model():
    

    def conv_block(x, _filter, _kernel, _stride):
        x = Conv2D(filters=_filter, kernel_size=_kernel, strides=_stride)(x)
        x = BatchNormalization()(x)
        x = Dropout(0.2)(x)
        x = Activation('relu')(x)
        return x

    def dense_block(x, _unit):
        x = Dense(units=_unit)(x)
        x = Dropout(0.5)(x)
        x = Activation('relu')(x)    
        return x

    def maxpool_block(x, _pool_size):
        x = MaxPool2D(pool_size=_pool_size)(x)
        x = Dropout(0.2)(x)
        return x

    input_layer=Input(shape=INPUT_SIZE)
    conv = conv_block(input_layer, 16, 3, (1,1))
    conv = maxpool_block(conv, (2,2))
    conv = conv_block(conv, 24, 3, (1,1))
    conv = conv_block(conv, 32, 5, (2,2))

    flat = Flatten()(conv)

    de = dense_block(flat, 32)
    output_layer = Dense(1, activation="sigmoid")(de)

    return Model(inputs= input_layer, outputs=output_layer)



class TrafficSign():
    def __init__(self):
        self.low_range = np.array([81, 159, 30])
        self.high_range = np.array([131, 255, 175])
        self.kernel = np.ones((3,3), np.uint8)
        self.modelPath =  os.path.join(os.path.dirname(os.path.realpath(__file__)), 'model', 'tfs_64-1.00.h5')
        self.model = build_model()
        self.model.summary()
        self.model.load_weights(self.modelPath)
        self.signal = False

    def detect_traffic_sign(self,frame):
        img, tf_sign = None, ''
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.low_range, self.high_range)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        mask = cv2.dilate(mask, self.kernel, iterations=1)
        contours = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        for c in contours:
            area = cv2.contourArea(c)
            if(area > 400):
                x,y,w,h = cv2.boundingRect(c)
                if (abs(w-h) < (w+h)/10):
                    img = frame[y:y+h,x:x+w]
                    imagePredict = util.resizeImage(img)
                    imagePredict = imagePredict / 255.0
                    start = time.time()
                    predicted = self.model.predict(np.array([imagePredict]))
                    predicted = round(predicted[0][0], 2) 
                    
                    if 0.0 <= predicted <= 0.2:
                        tf_sign = 'straight'
                    if 0.8 <= predicted <= 1.0:
                        tf_sign = 'turn'
                    print("{}\t{}\t{}".format(predicted, tf_sign, time.time() - start))
                       
        return img, tf_sign