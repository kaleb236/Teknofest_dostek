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
        self.orient = False
        self.obstacle = False
        self.stop_lane = True

        self.orientation_sub = rospy.Subscriber('orientation', Bool, self.orientation_callback)
        self.midpoint_sub = rospy.Subscriber('/points', linePoint, self.lane_function)
        self.stop_lane_sub = rospy.Subscriber('/stop_lane', Bool, self.stop_lane_callback)
        self.obstacle_sub = rospy.Subscriber('/obstacle', Bool, self.obstacle_callback)
        
        self.p = 1.3
        self.i = 2150
    
    def lane_function(self, msg):
        self.avaliable = msg.avaiable
        self.cx = msg.cx
        self.cy = msg.cy
        self.w = msg.w
        if self.avaliable:
            if self.orient or self.obstacle:
                rospy.logwarn('******Waiting for orientation******')
            
            else:
                self.main()
        
        else:
            if not self.stop_lane:
                self.cmd_vel(0.0, 0.0)
            else:
                rospy.logwarn('*******Lane following stopped********')
        

    def orientation_callback(self, orient):
        self.orient = orient.data
    
    def stop_lane_callback(self, lane):
        self.stop_lane = lane.data
    
    def cmd_vel(self, x, w):
        self.twist.linear.x = x
        self.twist.angular.z = w
        self.cmd_vel_pub.publish(self.twist)
    
    def obstacle_callback(self, obs):
        self.obstacle = obs.data
    
    def main(self):
        # if sys.getsizeof(self.cx) is not Empty:
        # rospy.loginfo('lane track is activated....')
        error = self.w / 4

        if self.cx >= (3*error) and self.cx <= (5*error):
            self.cmd_vel(0.12, 0)
            rospy.loginfo('move forward')
        
        elif self.cx > (5*error) and self.cx <= (7*error):
            self.cmd_vel(0.12, -0.2)
            rospy.loginfo('move right')
        
        elif self.cx >= (error) and self.cx < (3*error):
            self.cmd_vel(0.12, 0.2)
            rospy.loginfo('move left')
        
        elif self.cx > (7*error) and self.cx < (error):
            self.cmd_vel(0.0, 0)
            rospy.loginfo('stop')


if __name__ == '__main__':
    rospy.init_node('main_planner')
    planner = Planner()
    rospy.spin()
    # planner.main()
    