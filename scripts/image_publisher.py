#!/usr/bin/env python
#-*-coding: utf-8 -*-

import rospy, cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def publish_image():
    rospy.init_node('image_publisher', anonymous=True)

    pub = rospy.Publisher('/usb_cam/image_raw', Image, queue_size=0)

    cap = cv2.VideoCapture('/dev/video0')

    bridge = CvBridge()

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        ret, frame = cap.read()

        if ret:
            try:
                ros_msg = bridge.cv2_to_imgmsg(frame, 'bgr8')
                pub.publish(ros_msg)
            
            except CvBridgeError as e:
                rospy.logerr(e)
        
        rate.sleep()
    
    cap.release()
    rospy.loginfo('shutting down image_publisher node')
    
if __name__ == '__main__':
    try:
        publish_image()
    
    except rospy.ROSInterruptException:
        pass
