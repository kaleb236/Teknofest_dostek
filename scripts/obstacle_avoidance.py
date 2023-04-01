#!/usr/bin/env python
#-*-coding: utf-8 -*-

import rospy
import serial

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

ard = serial.Serial("/dev/ttyACM1", baudrate = 9600)

class Obstacle_avoidance:
    def __init__(self):
        self.vel = 0.12
        self.ang_vel = 0.3
        self.start_timer = True
        self.starting = 0
        self.obs_pub = rospy.Publisher('obstacle', Bool, queue_size=10)
        # self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.vel_msg = Twist()
    def main(self):       
        while not rospy.is_shutdown():
            try:
                distance = int(ard.readline().decode('utf-8'))
                if distance < 50 and distance > 10:
                    if self.start_timer:
                        self.start_timer = False
                        self.starting = rospy.get_time()
                        self.obs_pub.publish(True)
                        print('Starting obstacle avoidance')
                    
                    else:
                        if timeout > 20:
                            self.start_timer = True
                
                timeout = self.timeout(self.starting)
            
            except:
                print('could not read arduino')
    
    def timeout(self, scs):
        current = rospy.get_time()
        return (current - scs)

if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance', anonymous=True)
    Obstacle_avoidance().main()
    rospy.spin()