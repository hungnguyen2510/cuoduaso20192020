import cv2
import numpy as np

def nothing(x):
    pass

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

image = cv2.imread('1585548969.89.jpg')

while True:
    hsv_image = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    cv2.imshow('image',image)
    # cv2.imshow('hsv_image',hsv_image)
    

    hul= cv2.getTrackbarPos(hl,wnd)
    huh= cv2.getTrackbarPos(hh,wnd)
    sal= cv2.getTrackbarPos(sl,wnd)
    sah= cv2.getTrackbarPos(sh,wnd)
    val= cv2.getTrackbarPos(vl,wnd)
    vah= cv2.getTrackbarPos(vh,wnd)

    # print(hul)

    hsvl = np.array([hul, sal, val])
    hsvh = np.array([huh, sah, vah])
    mask = cv2.inRange(hsv_image, hsvl, hsvh)

    # res = cv2.bitwise_and(debugImage, debugImage, mask=mask)    
    cv2.imshow('hsv_img',hsv_image)         
    cv2.imshow('mask',mask)

    cv2.waitKey(1)
