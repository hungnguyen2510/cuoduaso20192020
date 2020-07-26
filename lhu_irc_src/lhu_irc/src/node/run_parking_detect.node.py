#!/usr/bin/env python
import rospy,cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge

from lib.parkingdetect import identify_area_parking

bridge = CvBridge()
frame = None

rospy.init_node('run_parking_detect', anonymous=True)


def callback_stream(ros_data):
    global bridge, frame
    frame = bridge.imgmsg_to_cv2(ros_data, "bgr8")
    cv2.imshow('ad',frame)
    cv2.waitKey(1)


######################################################################################
pub_parking = rospy.Publisher('/detect/lane/parking', Int32, queue_size=1)

rospy.Subscriber("/camera/rgb/image_rect_color", Image, callback_stream)
print('parking')
while not rospy.is_shutdown():
    if frame is not None:
        res = identify_area_parking(frame)
        print(res)
        pub_parking.publish(res)
    rospy.sleep(0.001)
    
