#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist

class Imu_class:
    def __init__(self):
        self.starting = False
        self.rotate_bool = False
        self.current_direction = 0.0
        self.directions = {
            'U': np.pi,
            'D': 0.0,
            'R': -np.pi/2,
            'L': np.pi/2
        }
        self.vel_msg = Twist()
        self.rotated_angle = np.array([0.5,0.5,0.5,0.5])
        self.sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.direction_sub = rospy.Subscriber('/direction_topic', String, self.direction_callback)
        self.pub = rospy.Publisher('orientation', Bool, queue_size=10)
        rospy.spin()
    
    def imu_callback(self, msg):
        current_orientation = np.array([msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z])
        if self.starting:
            self.rotated_angle = self.rotate(current_orientation)
            self.starting = False
            self.rotate_bool = True
        if self.rotate_bool:
            self.check_ornt(current_orientation)
    
    def direction_callback(self, direction):
        if direction.data == 'f':
            msg = Bool().data = True
            self.cmd_vel(0)
            self.pub.publish(msg)
            
        else:
            self.current_orientation = self.directions[direction.data]
            self.angle = self.current_orientation - self.current_direction
        
            if self.angle > 0:
                if abs(self.angle - 3*np.pi/2) <= 0.2:
                    self.angle = -np.pi/2
            else:
                if abs(self.angle + 3*np.pi/2) <= 0.2:
                    self.angle = np.pi/2

            self.starting = True
    
    def rotate(self, q):
        axis = np.array([0,0,1])
        axis = axis / np.linalg.norm(axis)

        angle = self.angle

        s = np.sin(angle / 2)
        c = np.cos(angle / 2)

        q_rot = np.array([c, axis[0] * s, axis[1] * s, axis[2] * s])
        q_new = self.quat_multipy([q_rot, q])

        return q_new
    
    def quat_multipy(self, q):
        result = np.array([1,0,0,0])
        for i in q:
            w1, x1, y1, z1 = result
            w2, x2, y2, z2, = i

            result = np.array([
                w1*w2 - x1*x2 - y1*y2 - z1*z2,
                w1*z2 + x1*w2 + y1*z2 - z1*y2,
                w1*y2 - x1*z2 + y1*w2 + z1*x2,
                w1*z2 + x1*y2 - y1*x2 + z1*w2 
            ])
        
        return result

    
    def cmd_vel(self, vel):
        vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.vel_msg.angular.z = vel
        vel_pub.publish(self.vel_msg)
    
    def check_ornt(self, q):
        diff = np.dot(q, self.rotated_angle)
        if diff >= 0.99:
            self.rotate_bool = False
            msg = Bool().data = False
            self.current_direction = self.current_orientation
            print('90 degree reached')
            self.cmd_vel(0)
        
        else:
            msg = Bool().data = True
            if self.angle > 0:
                self.cmd_vel(0.5)
            else:
                self.cmd_vel(-0.5)
            print(diff)
        self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('imu_node', anonymous=True)
    Imu_class()