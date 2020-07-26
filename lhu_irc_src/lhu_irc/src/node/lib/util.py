
import cv2
import os

INPUT_SIZE=(24,24,3)

def resizeImage(img):
    return cv2.resize(img, (INPUT_SIZE[0], INPUT_SIZE[1]), interpolation = cv2.INTER_AREA)

def normImage(img):
    return img / 255.0

def preProcessImage(img):
    img = resizeImage(img)
    img = normImage(img)
    return img

def readImage(path):
    return cv2.imread(path, 1)

def flipImage(img):
    return cv2.flip(img, 1)

def showImage(window, img):
    cv2.imshow(window, img)