#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
from lib.lanedetect import LaneDetect
from master.msg import Line
import time

bridge = CvBridge()
ld = LaneDetect()
frame = None

#----------------------------- MAIN ---------------------------------#
rospy.init_node('run_lane_detect', anonymous=True)

def callback_stream(ros_data):
    global frame
    frame = bridge.imgmsg_to_cv2(ros_data, "bgr8")

rospy.Subscriber("/camera/rgb/image_rect_color", Image, callback_stream)

pub_line = rospy.Publisher('/detect/lane', Line, queue_size=1)
    

while not rospy.is_shutdown():
    if frame is not None:
        start = time.time()
        line_left_properties, line_mid_properties, line_right_properties, line_dotted_properties = ld.detect(frame)
        line = Line(left = line_left_properties, mid= line_mid_properties, right = line_right_properties, dotted = line_dotted_properties)
        # print(line)
        # pub_line.publish(line)
        # print(time.time() - start)
     
    rospy.sleep(0.001)