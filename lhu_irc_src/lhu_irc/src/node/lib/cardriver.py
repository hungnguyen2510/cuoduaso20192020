#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32, Int32, Bool

from lib.Config import Config

config = Config()
config = config.get()

class CarDriver:
    def __init__(self, topic_angle, topic_speed):
        self.angle_pub = rospy.Publisher(topic_angle, Float32, queue_size=1)
        self.speed_pub = rospy.Publisher(topic_speed, Float32, queue_size=1)
        
        print("\nsubscribed to "+ topic_angle +" and " + topic_speed + "\n")

        self._angle = 0.0
        self._speed = 0.0
        
        self.a = 20
        self.b = 80

        self.stop_count = 50
        
        self.count = 0 # chu ky de xe dat vat toc thuc te
        self.current_speed = 0 # la toc do thuc te truyen xuong xe
        self.car_speed = config['min_speed'] # la toc do code tinh toan

    def reset(self):
        self.count = 0 # chu ky de xe dat vat toc thuc te
        self.current_speed = 0 # la toc do thuc te truyen xuong xe
        self.car_speed = config['min_speed'] # la toc do code tinh 

    def delay_time(self, speed):
        return  self.b - (speed * self.b / self.a)

    def control_speed_car(self):
        if(self.current_speed < self.car_speed):
            self.count += 1
            if(self.count >= self.delay_time(self.current_speed)):
                self.current_speed += 1
                self.count = 0
        else:
            if(self.current_speed > self.car_speed):
                self.count += 1
                if(self.count >= 1):
                    self.current_speed -= 5
                    self.count = 0

        if(self.car_speed == 0):
            self.stop_count -= 1
            if(self.stop_count > 0):
                self.current_speed = 8
            else:
                self.current_speed = config['min_speed']
                self.stop_count = 0
        else:
            if(self.current_speed < config['min_speed']):
                self.current_speed = config['min_speed']

    def updateSpeed(self, speed):
        self.car_speed = speed
        self.control_speed_car()
        self.speed_pub.publish(self.current_speed)
    
    def updateAngle(self, angle):
        angle = 60 if angle > 60 else angle
        angle = -60 if angle < -60 else angle
        self._angle =  angle
        self.angle_pub.publish(self._angle)