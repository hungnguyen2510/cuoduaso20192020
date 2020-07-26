#!/usr/bin/env python

import cv2
import numpy as np


class Libary:

    def mysort(self, array=[], index=0):
        less = []
        equal = []
        greater = []

        if len(array) > 1:
            pivot = array[0][index]
            for x in array:
                if(x[index] < pivot):
                    less.append(x)
                elif(x[index] == pivot):
                    equal.append(x)
                elif(x[index] > pivot):
                    greater.append(x)

            return self.mysort(less, index)+equal+self.mysort(greater, index)
        else:
            return array
            

    #################################################################################
    def determine_appear_item(self, arr_x):
        dictionary = {}

        for i in range(len(arr_x)):
            if(not self.check_exits_key(arr_x[i], dictionary)):
                dictionary.update({arr_x[i]: 1})
            else:
                value = dictionary.get(arr_x[i])
                value += 1
                dictionary.update({arr_x[i]: value})

        dictionary = dictionary.items()
        max_value = self.mysort(dictionary, index=1)[-1]
        return max_value[0]


    #################################################################################
    def check_exits_key(self, key, dictionary):
        if key in dictionary.keys():
            return True
        else:
            return False


    #################################################################################
    def fit_point_x(self, point_x):
        return np.linspace(point_x[0], point_x[-1], len(point_x), dtype=np.int16)


    #################################################################################
    def find_white_point(self, src):
        for i in range (src.shape[0]):
            for j in range (src.shape[1]):
                if src[i][j] == 255:
                    return True
        return False

    #################################################################################
    def have_item(self, arr, num=None):
        if(num is None):
            if(len(arr) > 0): return True
            return False
        else:
            if(len(arr) >= num): return True
            return False


    #################################################################################
    def change_point_with_num(self, points, num):
        temp = points[:]
        for i in range(len(temp)):
            temp[i] += num

        return temp