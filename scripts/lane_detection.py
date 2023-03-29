#!/usr/bin/env python
#-*-coding: utf-8 -*-

import rospy, cv2, logging
import cv_bridge
import numpy as np

from sensor_msgs.msg import Image
from teknofest_industrial_tech.msg import linePoint

class lane:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()

        self.image_topic = rospy.get_param("/image_topic", "/usb_cam/image_raw")
        self.show_image = rospy.get_param("/show_image", True)
        
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.fonksiyon) #it is for gazenbo
        # self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.fonksiyon) # it is for usb_cam
        self.pointPub = rospy.Publisher('/points', linePoint, queue_size=10)

    def fonksiyon(self, ros_goruntu):

        # converting topic to cv2 arrays
        cv_goruntu = self.bridge.imgmsg_to_cv2(ros_goruntu, 'bgr8') 

        hsv_goruntu = cv2.cvtColor(cv_goruntu, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(cv_goruntu, cv2.COLOR_BGR2GRAY)

        lower_yellow = np.array([0, 0, 0], np.uint8)
        upper_yellow = np.array([20, 20, 20], np.uint8)

        mask = cv2.inRange(hsv_goruntu, lower_yellow, upper_yellow)

        h, w, d = hsv_goruntu.shape

        w_half = w/2

  
        moment = cv2.moments(mask)
       
        if moment['m00'] > 0:
            cx = int(moment['m10']/moment['m00'])
            cy = int(moment['m01']/moment['m00'])

            cv2.circle(cv_goruntu, (cx, cy), 20, (0,255,100), -1)

            

            lpoints = linePoint()
            lpoints.cx = cx
            lpoints.cy = cy
            lpoints.w = w_half
            lpoints.avaiable = True
            self.pointPub.publish(lpoints)
            # rospy.loginfo("[INFO] lane detected")

        else:
            lpoints = linePoint()
            lpoints.avaiable = False
            self.pointPub.publish(lpoints)
            # rospy.logwarn("[WARN] lane cannot be detected")
        if self.show_image == True:
            cv2.imshow('mask', mask)
            cv2.imshow('merkez', cv_goruntu)
            cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node('lane_detection', anonymous=True)

    #activitng functions for ros debugger
    logger = logging.getLogger("rosout") #setting output broker of debug 
    logger.setLevel(logging.DEBUG)
    level = logger.getEffectiveLevel()

    obje = lane()
    rospy.spin()