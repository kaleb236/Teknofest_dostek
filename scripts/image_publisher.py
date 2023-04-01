#!/usr/bin/env python
#-*-coding: utf-8 -*-

import rospy, cv2
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from pyzbar import pyzbar

def publish_image():
    rospy.init_node('image_publisher', anonymous=True)

    pub = rospy.Publisher('/usb_cam/image_raw', Image, queue_size=0)
    qr_pub = rospy.Publisher('/qr_bool', Bool, queue_size=10)

    cap = cv2.VideoCapture('/dev/video1')
    # cap = cv2.VideoCapture(1)

    bridge = CvBridge()

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        ret, frame = cap.read()

        if ret:
            # try:
            barcodes = pyzbar.decode(frame)

            if barcodes:
                qr_pub.publish(True)
                rospy.logwarn('barcode detected')
            
            else:
                qr_pub.publish(False)
                # for barcode in barcodes:
                #     (x, y, w, h) = barcode.rect

                #     cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
            cv2.imshow('image', frame)
            cv2.waitKey(1)
            ros_msg = bridge.cv2_to_imgmsg(frame, 'bgr8')
            pub.publish(ros_msg)
            
            # except CvBridgeError as e:
            #     rospy.logerr(e)
        else:
            rospy.logwarn('lost camera signal')
        
        rate.sleep()
    
    cap.release()
    rospy.loginfo('shutting down image_publisher node')
    
if __name__ == '__main__':
    # try:
    publish_image()
    
    # except rospy.ROSInterruptException:
        # pass
