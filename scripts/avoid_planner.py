#!/usr/bin/env python
#-*-coding: utf-8 -*-

import rospy
import numpy as np

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class Obs_planner:
    def __init__(self):
        self.vel = 0.3
        self.ang_vel = 1
        self.s = 1
        self.vel_msg = Twist()
        self.sub = rospy.Subscriber('/obstacle', Bool, self.main)
        self.obs_pub = rospy.Publisher('obstacle', Bool, queue_size=10)
    def main(self, msg):
        if msg.data:
            ###### MOVE FORWARD ######
            print('back')
            self.cmd_vel(-self.vel, 0)
            rospy.sleep(self.timeout(self.vel, 0.1))
            print('turning')
            ### TURN 60 DEGREE ####
            self.cmd_vel(0, self.ang_vel)
            rospy.sleep(self.timeout(self.ang_vel, np.pi/2))
            ###### MOVE FORWARD ######
            print('foward')
            self.cmd_vel(self.vel, 0)
            rospy.sleep(self.timeout(self.vel, self.s))
            ######## TURN 60 DEGREE 
            print('turning')
            self.cmd_vel(0, -self.ang_vel)
            rospy.sleep(self.timeout(self.ang_vel, np.pi/2))
            ###### MOVE FORWARD ######
            print('foward')
            self.cmd_vel(self.vel, 0)
            rospy.sleep(self.timeout(self.vel, self.s))
            ######## TURN 600 DEGREE 
            print('turning')
            self.cmd_vel(0, -self.ang_vel)
            rospy.sleep(self.timeout(self.ang_vel, np.pi/2))
            ###### MOVE FORWARD ######
            print('foward')
            self.cmd_vel(self.vel, 0)
            rospy.sleep(self.timeout(self.vel, self.s))
            ### TURN 60 DEGREE ####
            print('turning')
            self.cmd_vel(0, self.ang_vel)
            rospy.sleep(self.timeout(self.ang_vel, np.pi/2)+0.5)
            print('stop')
            self.cmd_vel(0, 0)
            self.obs_pub.publish(False)

    def cmd_vel(self, vel, ang_vel):
        vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.vel_msg.linear.x = vel
        self.vel_msg.angular.z = ang_vel
        vel_pub.publish(self.vel_msg)
    
    def timeout(self, vel, s):
        return s / vel

if __name__ == '__main__':
    rospy.init_node('avoid_planner', anonymous=True)
    Obs_planner()
    rospy.spin()