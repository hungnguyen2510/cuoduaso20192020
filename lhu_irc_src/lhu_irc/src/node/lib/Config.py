#!/usr/bin/env python

import rospy
import yaml
import os

class Config():
    def __init__(self):
        self.MY_PATH = '/home/p2h/catkin_ws/src/lhu_irc/src/node'
        self.config = None
        self.run = None
        os.chdir(os.path.abspath(self.MY_PATH))

    def read_config(self):
        result = None
        with open("lhu.config.yaml", 'r') as stream:
            try:
                result = yaml.load(stream, Loader=yaml.SafeLoader)
                # print(yaml.load(stream))
                return result
            except yaml.YAMLError as exc:
                print(exc)

    def get(self):
        if(self.config is None):
            self.config = self.read_config()
            self.run = self.config['run']
            self.config = self.config[self.run]

            self.config.update({'MY_PATH': self.MY_PATH })
            return self.config
        
        return None