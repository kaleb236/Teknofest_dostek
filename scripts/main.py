#!/usr/bin/env python

import rospy
import sys

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from teknofest_industrial_tech.srv import *
from teknofest_industrial_tech.msg import linePoint

class Planner:
    def __init__(self):
        rospy.logwarn('[INFO] Planner is starting')

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.orient = True

        self.orientation_sub = rospy.Subscriber('orientation', Bool, self.orientation_callback)
        self.midpoint_sub = rospy.Subscriber('/points', linePoint, self.lane_function)
        
        self.p = 1.3
        self.i = 2150
    
    def lane_function(self, msg):
        self.avaliable = msg.avaiable
        self.cx = msg.cx
        self.cy = msg.cy
        self.w = msg.w
        if self.orient:
            rospy.logwarn('******Waiting for orientation******')
        
        else:
            self.main()
        

    def orientation_callback(self, orient):
        self.orient = orient.data
    
    def cmd_vel(self, x, w):
        self.twist.linear.x = x
        self.twist.angular.z = w
        self.cmd_vel_pub.publish(self.twist)
    
    def main(self):
        # if sys.getsizeof(self.cx) is not Empty:
        # rospy.loginfo('lane track is activated....')
        error = self.w / 4

        if self.cx >= (3*error) and self.cx <= (5*error):
            self.cmd_vel(0.3, 0)
            rospy.loginfo('move forward')
        
        elif self.cx > (5*error) and self.cx <= (7*error):
            self.cmd_vel(0, -0.4)
            rospy.loginfo('move right')
        
        elif self.cx >= (error) and self.cx < (3*error):
            self.cmd_vel(0.0, 0.4)
            rospy.loginfo('move left')
        
        elif self.cx > (7*error) and self.cx < (error):
            self.cmd_vel(0.0, 0)
            rospy.loginfo('stop')


if __name__ == '__main__':
    rospy.init_node('main_planner')
    planner = Planner()
    rospy.spin()
    # planner.main()
    