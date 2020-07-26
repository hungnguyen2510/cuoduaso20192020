#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
# from trafficsign import TrafficSign
from trafficsign_trt import CNN


bridge = CvBridge()
# traffic_sign = TrafficSign()
cnn = CNN()

frame = None

rospy.init_node('run_traffic_sign_detect', anonymous=True)


def callback_stream(ros_data):
    global frame
    frame = bridge.imgmsg_to_cv2(ros_data, "bgr8")


######################################################################################
pub_traffic_sign = rospy.Publisher('/detect/traffic_sign/signal', String, queue_size=1)
# traffic_sign.pub_extract_gray = rospy.Publisher('/debug/image/extract_ts_gray', Image, queue_size=1)

rospy.Subscriber("/camera/rgb/image_rect_color", Image, callback_stream)

while not rospy.is_shutdown():
    if frame is not None:
        tfs = cnn.process(frame.copy())
        # pub_traffic_sign.publish(tfs)

    rospy.sleep(0.01)
