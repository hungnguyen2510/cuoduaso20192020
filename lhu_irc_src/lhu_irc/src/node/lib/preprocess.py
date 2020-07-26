#!/usr/bin/env python3

import cv2
import numpy as np
from matplotlib import pyplot as plt
from lib.Config import Config

config = Config()
config = config.get()

kernel_sharpening = np.array([[-1,-1,-1], 
                            [-1, 9,-1],
                            [-1,-1,-1]])

h_sky_view = 120
h_sky_view -= config['increase_sky_view']

class Preprocess:

    def sky_view(self, img):
        src_point = np.float32([
                                [0 + config['sky_top'], h_sky_view], [320 - config['sky_top'], h_sky_view], 
                                [0 + config['sky_bot'], img.shape[0]], [320 - config['sky_bot'], img.shape[0]]
                                ])

        dst_point = np.float32([
                                [src_point[0][0] - config['increase_sky_top'], 0], [src_point[1][0] + config['increase_sky_top'], 0],
                                [src_point[0][0] + config['increase_sky_bot'], img.shape[0]], [src_point[1][0] - config['increase_sky_bot'], img.shape[0]]
                                ])

        M = cv2.getPerspectiveTransform(src_point, dst_point)
        result = cv2.warpPerspective(img,M,(img.shape[1], img.shape[0]))
        M_inv = cv2.getPerspectiveTransform(dst_point, src_point)
    
        return M_inv, result

    #############################################################################
    def increase_white_after_skyview(self, src):
        threshold       = cv2.threshold(src, 50, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)[1]
        erosion         = cv2.erode(threshold,cv2.getStructuringElement(cv2.MORPH_RECT,(1,3)),iterations = 1)
        # closing         = cv2.morphologyEx(erosion, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT,(3,3)))
        dilate          = cv2.dilate(erosion,cv2.getStructuringElement(cv2.MORPH_RECT,(2,7)),iterations = 1)
        # openning        = cv2.morphologyEx(dilate, cv2.MORPH_TOPHAT, cv2.getStructuringElement(cv2.MORPH_RECT,(21,5)))

        return dilate


    #############################################################################
    def equalize_histogram(self, img):
        img = img.astype(np.uint8)
        image_yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
        clahe = cv2.createCLAHE(clipLimit=10.0, tileGridSize=(8,8))
        image_yuv[:,:,0] = clahe.apply(image_yuv[:,:,0])
        image_output = cv2.cvtColor(image_yuv, cv2.COLOR_YUV2BGR)

        return image_output


    #############################################################################
    def auto_contrast(self, img, clipLimit):
        stack_img = cv2.split(img)
        clahe = cv2.createCLAHE(clipLimit=clipLimit,tileGridSize=(8,8))
        stack_img[0] = clahe.apply(stack_img[0])
        stack_img[1] = clahe.apply(stack_img[1])
        stack_img[2] = clahe.apply(stack_img[2])
        new_image = cv2.merge(stack_img)

        return new_image


    #############################################################################
    def calc_hist(self, img):
        a,b  = 0,0
        gray_img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        hist = cv2.calcHist([gray_img],[0],None,[256],[0,256])
        max_I = np.argmax(hist)
        min_I = np.argmin(hist) 
        a = (255//(max_I - min_I))
        b = (-min_I*a)
        return a, b


    #############################################################################
    def find_rgb_white(self, src):
        lower = np.array([0,0,0],dtype=np.uint8)
        upper = np.array([255,255,255],dtype=np.uint8)
        mask =  cv2.inRange(src,lower,upper)
        rgb_w = cv2.bitwise_and(src,src,mask).astype(np.uint8)
        rgb_w = cv2.cvtColor(rgb_w,cv2.COLOR_RGB2GRAY)

        output = np.zeros_like(src)
        output[(rgb_w >= 200) & (rgb_w <= 255)] = 255
        output = cv2.cvtColor(output, cv2.COLOR_RGB2GRAY)
        return output


    #############################################################################
    def find_hsv_white(self, src, sensitivity=50):
        hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        
        sensitivity = sensitivity
        lower_white = np.array([0,0,255-sensitivity])
        upper_white = np.array([255,sensitivity,255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        return mask


    #############################################################################
    def find_edge_lane(self, src, di_kernel=(1,1)):
        blur = cv2.GaussianBlur(src, (5,5), 0)
        canny = cv2.Canny(blur, 200, 400, None, 3, False)

        canny = cv2.dilate(canny,cv2.getStructuringElement(cv2.MORPH_RECT,di_kernel),iterations = 1)
        return canny


    #############################################################################
    def enhance_brightness(self, img, alpha=2, beta=0):
        img_temp = cv2.addWeighted(img, alpha, np.zeros(img.shape, img.dtype), 0, beta)
        return img_temp


    #############################################################################
    def extract_horizontal_line(self, img, kenel=(6, 50)):
        linek = np.zeros(kenel,dtype=np.uint8)
        linek[kenel[0]//2-1,...]=1

        horizontal_line = cv2.morphologyEx(img, cv2.MORPH_OPEN, linek ,iterations=1)

        return horizontal_line


    #############################################################################
    def extract_vertical_line(self, img, kenel=(5, 3)):
        linek = np.zeros(kenel,dtype=np.uint8)
        linek[...,kenel[1]//2-1]=1

        vertical_line = cv2.morphologyEx(img, cv2.MORPH_OPEN, linek ,iterations=1)

        # cv2.imshow('vertical_line', vertical_line)
        return vertical_line


    #############################################################################
    def auto_canny(self, img, sigma=0.33):
        v = np.median(img)
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))

        return cv2.Canny(img, lower, upper)


    #############################################################################
    def morph_lane(self, img):
        erosion = cv2.erode(img,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(1,2)),iterations = 1)
        dilate = cv2.dilate(img,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2,6)),iterations = 1)

        return dilate
    

    #############################################################################
    def remove_noise(self, img):
        num_sub_border_mask = 1
        mask = np.zeros_like(img)
        mask[num_sub_border_mask:img.shape[0]-num_sub_border_mask, num_sub_border_mask:img.shape[1]-num_sub_border_mask] = 255

        img = cv2.bitwise_and(img, mask)

        im2, contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            if(area < 50): # change area
                cv2.drawContours(img,[contours[i]],0,(0,0,0),-1)
        
        return img


    #############################################################################
    def handle(self, img):
        img_roi = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        hist = cv2.calcHist([img_roi],[0],None,[256],[0,256])
        # hist = hist.astype(int)

        low_hist = hist[:85]
        mid_hist = hist[85:170]
        high_hist = hist[170:255]

        min_mid_hist = np.argmin(mid_hist) + 85
        min_high_hist = np.argmin(high_hist) + 170

        alpha = (min_high_hist//min_mid_hist)*1.2
        beta = ((min_high_hist)*-1)

        new = self.enhance_brightness(img_roi, alpha=alpha, beta=beta)

        _, thre = cv2.threshold(new, 1, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        # plt.hist(img_roi.ravel(),256,[0,256]); 
        # plt.show()

        # horizontal_line = self.extract_horizontal_line(thre)
        # thre-=horizontal_line

        # vertical_line = self.extract_vertical_line(thre)
        # thre-=vertical_line

        return thre


    #############################################################################
    def handle2(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        canny = self.find_edge_lane(gray, di_kernel=(2,2))

        return canny


    #############################################################################
    def handle3(self, img):
        hist = cv2.calcHist([img],[0],None,[256],[0,256])
        low_hist = hist[:85]
        mid_hist = hist[85:170]
        high_hist = hist[170:255]

        mean_low = np.median(low_hist)
        mean_high = np.median(high_hist)

        if(mean_low > mean_high and abs(mean_low - mean_high) > 100):
            img = self.auto_contrast(img, clipLimit=2)

        white_lane = self.find_rgb_white(img)
        return white_lane

    #############################################################################
    def handle4(self, img):
        hist = cv2.calcHist([img],[0],None,[256],[0,256])
        low_hist = hist[:85]
        mid_hist = hist[85:170]
        high_hist = hist[170:255]

        mean_low = np.median(low_hist)
        mean_high = np.median(high_hist)

        print(mean_low, mean_high)
        if(mean_low > mean_high and abs(mean_low - mean_high) > 150):
            img = self.auto_contrast(img, clipLimit=2)

        white_lane = self.find_hsv_white(img, sensitivity=100)
        return white_lane

    #############################################################################
    def handle5(self, img):
        a,b = self.calc_hist(img)
        img = self.auto_contrast(img, clipLimit=a)

        white = self.find_rgb_white(img)
        gray = cv2.cvtColor(white, cv2.COLOR_BGR2GRAY)
        horizontal_line = self.extract_horizontal_line(gray)
        gray-=horizontal_line

        return gray