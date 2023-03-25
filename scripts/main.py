#!/usr/bin/env python3.8

import rospy
import sys

from queue import Empty
from teknofest_industrial_tech.srv import *
from teknofest_industrial_tech.msg import linePoint

class Planner:
    def __init__(self):
        rospy.logwarn('[INFO] Planner is starting')

        self.midpoint_sub = rospy.Subscriber('/points', linePoint, self.lane_function)
        self.p = 1.3
        self.i = 2150
    
    def lane_function(self, msg):
        self.avaliable = msg.avaiable
        self.cx = msg.cx
        self.cy = msg.cy
        # self.w = 320
        self.w = msg.w
        self.main()
    
    def main(self):
        if sys.getsizeof(self.cx) is not Empty:
            # rospy.loginfo('lane track is activated....')
            error = self.w / 4
            # error = self.cx - self.w

            # rospy.loginfo(f'the error is: {error}')

            if self.cx >= (3*error) and self.cx <= (5*error):
                rospy.loginfo('move forward')
            
            elif self.cx > (5*error) and self.cx <= (7*error):
                rospy.loginfo('move right')
            
            elif self.cx >= (error) and self.cx < (3*error):
                rospy.loginfo('move left')
            
            elif self.cx > (7*error) and self.cx < (error):
                rospy.loginfo('move forward')

            # if error >= -20 and error <= 20:
            #     rospy.loginfo('move forward')
            
            # elif error > 20 and error <= 80:
            #     rospy.loginfo('turn right')

            # elif error > -80 and error < -20:
            #     rospy.loginfo('turn left')
            
            # elif error < -80 or error > 80:
            #     rospy.loginfo('stop')


if __name__ == '__main__':
    rospy.init_node('main_planner')
    planner = Planner()
    rospy.spin()
    # planner.main()
    