#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32, Int32, Int32MultiArray, Bool
from cv_bridge import CvBridge

import cv2
import numpy as np

from lib.cardriver import CarDriver
from lib.preprocess import Preprocess
from lib.libary import Libary

import time
from simple_pid import PID
import csv

pid = PID(1, 0., 0.002, setpoint=0)
pid.output_limits = (-60, 60)

p = Preprocess()
l = Libary()

#----------------------------------- LaneDetect -----------------------------------#
class LaneDetect:
    def __init__(self, config):
        self.config = config

        self.half_h = 240//2

        #calc steer angle
        self.prev_k = 0
        self.cur_k = 0
        self.M = 2

        self.left = -1
        self.right = 1
    
        #sky view
        self.w_sky = self.config['w_sky']
        self.h_sky = self.config['h_sky']
        self.increase_sky_view = self.config['increase_sky_view']
        self.w_view = self.w_sky//2
        self.half_w_sky = self.w_sky//2

        self.last_width_road = 160
        self.last_left = 0
        self.last_right = 320

        # angle 
        self.cur_angle = 0

        # speed
        self.cur_speed = 0
        self.min_speed = self.config['min_speed']
        self.max_speed = self.config['max_speed']
        self.sub_num = 1/5.0

        # sliding
        self.w_sliding = self.config['w_sliding']
        self.h_sliding = self.config['h_sliding']

        self.limit_w_sliding = 100
        self.limit_h_sliding = 5

        self.is_find_lane = False
        self.obstacle_side = 0
        self.time_avoid_obstacle = 0
        self.p_mid_for_obstacle = [160, 120]
    
    #################################################################################
    def reset_not_find_lane(self):
        self.cur_angle = 0
        self.last_left = 0
        self.last_right = 320
        self.is_find_lane = False
        self.w_view = self.w_sky//2

    #################################################################################
    def update_last_point(self, side, points=()):
        if(side == self.left):
            self.last_left = int(np.mean(points[0]))

        if(side == self.right):
            self.last_right = int(np.mean(points[0]))

    #################################################################################
    def calc_cur_angle(self, points, move_mid, num_not_find, is_mid=False):

        if(not l.have_item(points) and not is_mid):
            return num_not_find

        mid = int(np.median(points))
        angle = (mid + move_mid) - self.w_view
        is_find_lane = True
        angle =  60 if angle > 60 else angle
        angle = -60 if angle < -60 else angle

        return angle

    #################################################################################
    def calc_angle(self, point_m, move_mid):
        self.cur_k = (point_m + move_mid) - self.w_view

        if(self.prev_k - self.cur_k >= 0):
            if self.M < 0:
                self.M = 0
            if self.M < 60:
                self.M *= 2

        if(self.prev_k - self.cur_k < 0):
            if self.M > 0:
                self.M = 0
            if self.M > -60:
                self.M /= 2
              
        if(self.cur_k > 0):
            result = self.cur_k if abs(self.cur_k) < 2 else self.cur_k + self.M
        else:
            result = self.cur_k if abs(self.cur_k) < 2 else self.cur_k - self.M
        self.prev_k = self.cur_k

        return result

    #################################################################################
    def determine_pos_lane(self, pos_lane, pos_sliding, mid_windown):
        # pos_lane (x, y, w, h)
        # pos_sliding (x, y)
        is_pos_left, is_pos_right = False, False

        if(pos_sliding[0] < mid_windown):
            if(pos_lane[3] > self.h_sky//2.2):
                is_pos_left = True

        if(pos_sliding[0] > mid_windown):
            if(pos_lane[3] > self.h_sky//2.2):
                is_pos_right = True

        return is_pos_left, is_pos_right
    

    #################################################################################
    def get_final_mask_and_pos_sliding(self, src_binary, resources=[]):
        # resources [(box_contour, height, pos_sliding, height_for_loop_sliding)]
        mask = None
        pos_sliding = None
        box_height = None

        if(l.have_item(resources)):
            mask = np.zeros_like(src_binary)
            res_sort_max = l.mysort(resources, index=1)[-1]
            pos_sliding = res_sort_max[2] # x, y
            box_height = res_sort_max[3]
            cv2.drawContours(mask,[res_sort_max[0]],0,(255,255,255),-1)
            mask = cv2.bitwise_and(src_binary, mask)

        return mask, pos_sliding, box_height


    #################################################################################
    def find_contour_lane(self, src_binary, src_rgb, area_size, mid_windown):
        contours = cv2.findContours(src_binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)[0]

        pos_left, pos_right = None, None
        mask_left, mask_right = None, None
        on_left, on_right = [], [] # (box_contour, height, pos_sliding)

        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])

            if(area > area_size):                
                rect = cv2.minAreaRect(contours[i])
                (x, y), (w, h), theta = rect
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                _x, _y, _w, _h = cv2.boundingRect(contours[i])
                
                if(self.config['debug_findcontours'] and src_rgb is not None):
                    cv2.drawContours(src_rgb,[box],0,(255,255,0),1)
                    cv2.circle(src_rgb,(box[0][0], box[0][1]), 5, (0,255,255), -1)
                    cv2.rectangle(src_rgb,(_x, _y),(_x+_w,_y+_h),(0,255,0),1)

                if(w > h):
                    w, h = h, w
                    
                pos_lane = np.array([x, y, w, h, _h]).astype(np.int16) # x, y is point center of cnt

                # find point x for sliding
                index_max = np.argmax(contours[i], axis=0)[0][1]
                pos_sliding = contours[i][index_max][0] #[x, y]
                
                is_pos_left, is_pos_right = self.determine_pos_lane(pos_lane=pos_lane,
                                                                    pos_sliding=pos_sliding,
                                                                    mid_windown=mid_windown)

                if(is_pos_left): 
                    on_left.append((box, pos_lane[3], pos_sliding, pos_lane[4]))

                if(is_pos_right):
                    on_right.append((box, pos_lane[3], pos_sliding, pos_lane[4]))

        # get max height contour
        mask_left, pos_left, box_height_left = self.get_final_mask_and_pos_sliding(src_binary=src_binary, 
                                                                    resources=on_left)

        mask_right, pos_right, box_height_right = self.get_final_mask_and_pos_sliding(src_binary=src_binary, 
                                                                    resources=on_right)

        return {'mask_left': mask_left, 'left': pos_left, 'box_height_left': box_height_left,
                'mask_right':mask_right, 'right': pos_right, 'box_height_right': box_height_right}


    #################################################################################
    def sliding_window(self, img_binary, img_rgb, pos_start=[], height_windows=5, width_window=50, height_box=0):
        w, h = width_window, height_windows
        x = 0 if pos_start[0] - w//2 < 0 else pos_start[0] - w//2
        y = pos_start[1] - h
        list_pos_x = []
        x_result, y_result = [], []
        loop = int(height_box / h)
        is_not_find = False

        for i in range(0, loop, 1):

            if(is_not_find):
                h = 5
                w = 100

            list_pos_x.append(x)
            M = cv2.moments(img_binary[y:y+h, x:x+w])

            if(M['m00'] != 0):
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                x_result.append(x+cx)
                y_result.append(y+cy)

                if(self.config['debug_sliding_window'] and img_rgb is not None):
                    cv2.circle(img_rgb,(x+cx, y+cy), 2, (0,0,255), -1)
                    cv2.rectangle(img_rgb,(x,y),(x+w,y+h),(0,0,255),1)

                x = 0 if list_pos_x[i] + (cx - w//2) < 0 else list_pos_x[i] + (cx - w//2)

                is_not_find = False
                w, h = width_window, height_windows
            else:
                is_not_find = True

            y -= h
   
        return x_result, y_result, loop


    #################################################################################
    def determine_curve_lane(self, src_binary, x_point1, x_point2):
        num_division = self.config['num_curve'] # so dung de tang move_mid theo do cong
        diff_abs = abs(x_point1 - x_point2)
        curve_side = 0
        move_mid = 0

        #----------------------------------------------------------------#
        if(x_point1 - x_point2 > 0):
            curve_side = self.left
            move_mid = int((diff_abs/num_division)) * -1

        if(x_point1 - x_point2 < 0):
            curve_side = self.right
            move_mid = int((diff_abs/num_division)) * 1

        return curve_side, diff_abs, move_mid

    #################################################################################
    def get_point_with_num(self, img_rgb, points=(), num=0):
        res_points = []

        if(l.have_item(points)):
            res_points = l.change_point_with_num(points[0], num)
            if(self.config['debug_draw_point_side'] and img_rgb is not None):
                for i in range(len(res_points)):
                    cv2.circle(img_rgb,(res_points[i], points[1][i]), 3, (255,0,0), -1)

        return res_points
    
    #################################################################################
    def get_point_mid_side(self, img_rgb, x_left, x_right):
        mid = ((x_left + x_right)/2)

        if(self.config['debug_draw_point_mid'] and img_rgb is not None):
            cv2.circle(img_rgb,(mid, 120), 5, (255,0,255), -1)

        return [mid]

    #################################################################################
    def check_reliability_lane(self, points=[], percent=0.85):
        if(not l.have_item(points[0])):
            return False

        if(len(points[0])*1. >= points[2]*percent):
            return True
        return False


    #################################################################################
    def tracking_side(self, img_rgb, point_left=(), point_right=(), have_left=None, curve_side_of_left=0, move_mid_left=0, have_right=None, curve_side_of_right=0, move_mid_right=0):

        point_x_left = []
        point_x_right = []
        curve_left, curve_right = 0, 0

        mid = self.get_point_mid_side(img_rgb, self.last_left, self.last_right)
        if(have_left):
            point_x_left = self.get_point_with_num(img_rgb, point_left, 50)
            curve_left = curve_side_of_left[0] * curve_side_of_left[1]

        if(have_right):
            point_x_right = self.get_point_with_num(img_rgb, point_right, -50)
            curve_right = curve_side_of_right[0] * curve_side_of_right[1]

        return (point_x_left, curve_left), (point_x_right, curve_right), mid

    
    #################################################################################
    def get_point_by_sliding(self, img_rgb, side, mask_and_point):
        curve_side, diff_abs = 0, 0
        have_lane = False
        move_mid = 0

        x_point, y_point, numbers = self.sliding_window(img_binary=mask_and_point[0], 
                                                        img_rgb=img_rgb, 
                                                        pos_start=mask_and_point[1], 
                                                        height_windows=self.h_sliding, 
                                                        width_window=self.w_sliding,
                                                        height_box=mask_and_point[2])
        # kiem tra duong cong
        if(l.have_item(x_point)):
            curve_side, diff_abs, move_mid = self.determine_curve_lane(src_binary=img_rgb,
                                        x_point1=x_point[0], # diem y lon nhat
                                        x_point2=x_point[-1]) # diem y nho nhat

            self.increase_w_sliding_with_curve_road(diff_abs, x_point)
            have_lane = self.check_reliability_lane(points=(x_point, y_point, numbers), percent=0.8)

            if(have_lane):
                self.update_last_point(side=side, points=(x_point, y_point))

        return x_point, y_point, have_lane, (curve_side, diff_abs), move_mid


    #################################################################################
    def increase_w_sliding_with_curve_road(self, diff_abs, x_point):
        if(diff_abs > self.config['num_curve']):
            self.w_sliding = self.limit_w_sliding if(self.w_sliding > self.limit_w_sliding) else self.w_sliding + (diff_abs/80)
            self.h_sliding = self.limit_h_sliding if(self.h_sliding < self.limit_h_sliding) else self.h_sliding - (diff_abs/80)

        # reset when finished
        if(diff_abs < self.config['num_curve']):
            self.w_sliding = self.config['w_sliding']
            self.h_sliding = self.config['h_sliding']

    ########### junction #############################################################
    def check_junction(self, img_binary):
        non_zeros = np.nonzero(img_binary)
        if(not l.have_item(non_zeros[0])):
            return True
        return False

    ############## junction #############################################################
    def check_finish_turn_junc(self, side, have_left, have_right):
        if(abs(self.cur_angle) < 10 and (have_left and have_right)):
            return True
        return False

    ############## junction #############################################################
    def determine_loss_lane(self, img_binary, img_rgb, point, w_windown=120, range=[200, 250]):
        w = w_windown
        h = range[1] - range[0]
        x = point if point - w//2 < 0 else point - w//2
        y = range[0] - h//2
        mask_lane = img_binary[y:y+h, x:x+w]

        if(self.config['debug_junc'] and img_rgb is not None):
            cv2.imshow('mask_lane', mask_lane)
            cv2.rectangle(img_rgb,(x,y),(x+w,y+h),(255,0,255),1)
            cv2.circle(img_rgb,(point, range[0]), 3, (0,255,255), -1)

        non_zeros = np.nonzero(mask_lane)
        if(not l.have_item(non_zeros[0])):
            return True
        return False
        
    ############## junction #############################################################
    def go_straight(self):
        pass

    #################################################################################
    def update_obstacle(self, obstacle):
        if(obstacle != 0):
            self.obstacle_side = obstacle
            self.time_avoid_obstacle = time.time()

        if((time.time() - self.time_avoid_obstacle) >= 1.5 and obstacle != 0):
            self.obstacle_side = 0
            self.time_avoid_obstacle = 0

    #################################################################################
    def get_point_for_obtascle_detect(self, img_rgb, M):
        output = cv2.perspectiveTransform(np.array([[[self.cur_mid + self.move_mid, self.half_h-self.increase_sky_view]]], dtype="float32"), M)
        self.p_mid_for_obstacle = output.flatten().astype(np.int16)
        if(self.config['debug_final'] and not self.config['turn_off_all_debug']):
            cv2.circle(img_rgb,(self.p_mid_for_obstacle[0], self.p_mid_for_obstacle[1]), 5, (0,0,255), -1)

    #################################################################################
    def process_lane_detect(self, src, obstacle):
        x_point_left, y_point_left, have_left, curve_side_of_left = [], [], False, (0, 0)
        x_point_right, y_point_right, have_right, curve_side_of_right = [], [], False, (0, 0)
        move_mid_left, move_mid_right = 0, 0
        mid = 160
       
        handle_line         = p.handle2(src)
        M, sky_view         = p.sky_view(handle_line)
        sky_view            = p.increase_white_after_skyview(sky_view)
        sky_view_rgb        = None

        if(not self.config['turn_off_all_debug']):
            _, sky_view_rgb = p.sky_view(src)

        # self.update_obstacle(obstacle)
        
        # STEP 1: -------------------------------------------------------------
        pos_sliding = self.find_contour_lane(src_binary=sky_view, 
                                            src_rgb=sky_view_rgb, 
                                            area_size=100,
                                            mid_windown=self.half_w_sky)

        # STEP 2: -------------------------------------------------------------
        if(pos_sliding['left'] is not None):
            [x_point_left, 
            y_point_left, 
            have_left,
            curve_side_of_left,
            move_mid_left] = self.get_point_by_sliding(img_rgb=sky_view_rgb, 
                                                        side=self.left,
                                                        mask_and_point=(pos_sliding['mask_left'], 
                                                                    pos_sliding['left'],
                                                                    pos_sliding['box_height_left'])
                                                        )

        if(pos_sliding['right'] is not None):
            [x_point_right, 
            y_point_right, 
            have_right,
            curve_side_of_right,
            move_mid_right] = self.get_point_by_sliding(img_rgb=sky_view_rgb, 
                                                            side=self.right, 
                                                            mask_and_point=(pos_sliding['mask_right'], 
                                                                        pos_sliding['right'],
                                                                        pos_sliding['box_height_right'])
                                                        )

        # STEP 3: -------------------------------------------------------------
        left_final, right_final, mid = self.tracking_side(img_rgb=sky_view_rgb,
                                                            point_left=(x_point_left, y_point_left),
                                                            point_right=(x_point_right, y_point_right),
                                                            have_left=have_left,
                                                            move_mid_left=move_mid_left,
                                                            curve_side_of_left=curve_side_of_left,
                                                            have_right=have_right,
                                                            curve_side_of_right=curve_side_of_right,
                                                            move_mid_right=move_mid_right)

        angle_left = self.calc_cur_angle(left_final[0], move_mid_left, -333)
        angle_right = self.calc_cur_angle(right_final[0], move_mid_right, 333)
        angle_mid = self.calc_cur_angle(mid, 0, None, is_mid=True)
        # angle_mid = self.calc_cur_angle(right_final[0],0,None,is_mid=True)

        if(self.config['debug_calibrate_camera']):
            calibrate = np.copy(src)
            cv2.circle(calibrate,(160, 120), 5, (0,0,255), -1)

            cv2.circle(calibrate,(0+self.config['sky_top'], 120-self.config['increase_sky_view']), 4, (0,0,255), -1)
            cv2.circle(calibrate,(320-self.config['sky_top'], 120-self.config['increase_sky_view']), 4, (0,0,255), -1)
            cv2.circle(calibrate,(0+self.config['sky_bot'], 240), 4, (0,0,255), -1)
            cv2.circle(calibrate,(320-self.config['sky_bot'], 240), 4, (0,0,255), -1)

            cv2.line(calibrate, (0,120-self.config['increase_sky_view']), (320, 120-self.config['increase_sky_view']), (255,0,0), 2)
            cv2.imshow('calibrate', calibrate)

        #############################################################################
        if(self.config['debug_final'] and not self.config['turn_off_all_debug']):
            cv2.imshow('sky_view', sky_view)
            cv2.line(sky_view_rgb, (self.w_view, 0), (self.w_view, self.h_sky), (255,0,0), 2)
            cv2.imshow('sky_view_rgb', sky_view_rgb)
            cv2.waitKey(1)

        return [angle_left, left_final[1], angle_right, right_final[1], angle_mid, mid[0]]