#!/usr/bin/env python3
#-*-coding: utf-8 -*-

import rospy
import serial

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

ard = serial.Serial("/dev/ttyACM0", baudrate = 9600)

class Obstacle_avoidance:
    def __init__(self):
        self.pub = rospy.Publisher('orientation', Bool, queue_size=10)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.vel_msg = Twist()
    def main(self):       
        while not rospy.is_shutdown():
            try:
                distance = int(ard.readline().decode('utf-8'))
                if distance < 20:
                    self.pub.publish(True)
                    print('Starting obstacle avoidance')
            
            except:
                print('could not read arduino')

    def cmd_vel(self, vel):
        self.vel_msg.angular.z = vel
        self.vel_pub.publish(self.vel_msg)

if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance')
    Obstacle_avoidance().main()
    rospy.spin()