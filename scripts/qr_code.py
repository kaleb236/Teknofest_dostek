#!/usr/bin/env python3
#-*-coding: utf-8 -*-

import cv2
import cv_bridge
import rospy

from pyzbar import pyzbar
from std_msgs.msg import Bool, String

class qr_reader:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/qr_bool', Bool, self.qr_callback)
        self.stop_lane_pub = rospy.Publisher('stop_lane', Bool, queue_size=10)
        self.pub = rospy.Publisher('qr_state', String, queue_size=10)
        self.cnt = 0
        self.start_time = True
        self.starting = 0
        self.wait_time = 0
    
    def qr_callback(self, state):
        if state.data == True:
            if self.start_time:
                if self.cnt == 1:
                    self.wait_time = 12
                    print('second qr detected sending exception')
                    self.stop_lane_pub.publish(True)
                
                else:
                    self.wait_time = 2
                    print('qr detected sending')
                self.starting = rospy.get_time()
                self.start_time = False

        timeout = self.timeout(self.starting)
        if not self.start_time and timeout > self.wait_time:
            self.cnt +=1 
            self.start_time = True
            if self.cnt == 2:
                print('sending turning')
                self.cnt = 0
                self.stop_lane_pub.publish(False)
                self.pub.publish('t')
    
    def timeout(self, scs):
        current = rospy.get_time()
        return (current - scs)
            
if __name__ == '__main__':
    rospy.init_node('qr_reader', anonymous=True)
    qr_reader()
    rospy.spin()
    # main()