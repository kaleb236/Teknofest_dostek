#!/usr/bin/env python3
#-*-coding: utf-8 -*-

import rospy

from std_msgs.msg import String

class Global_planner:
    def __init__(self):
        self.dir_index = 0
        self.path_index = 0
        self.directions = ['R', 'D', 'D', 'L', 'U', 'U']
        self.loading_state = True
        self.full_path = []
        self.pub = rospy.Publisher('direction_topic', String, queue_size=10)
        self.sub = rospy.Subscriber('/qr_state', String, self.qr_callback)
        self.mqtt_sub = rospy.Subscriber('/mqtt_server', String, self.mqtt_callback)
    
    def qr_callback(self, state):
        if state.data == 't': 
            if len(self.directions) > self.dir_index:
                self.pub.publish(self.directions[self.dir_index])
                rospy.logwarn('********SENDING DIRECTION*******')

            elif len(self.directions) == self.dir_index:
                msg = String().data = 'f'
                self.pub.publish(msg)
                if self.loading_state:
                    rospy.logwarn('********LOADING*******')
                    self.loading_state = False
                else:
                    rospy.logwarn('********UNLOADING*******')
                    self.loading_state = True
                rospy.sleep(10)
                self.path_index+=1
                if len(self.full_path) > (self.path_index + 1):
                    self.directions = self.full_path[self.path_index]
                    self.dir_index = 0
                    if len(self.directions) > 0:
                        self.pub.publish(self.directions[self.dir_index])

            self.dir_index += 1
    
    def mqtt_callback(self, directions):
        self.dir_index = 0
        self.path_index = 0
        self.full_path = []
        self.loading_state = True
        paths = directions.data.split(';')
        for path in paths:
            self.full_path.append(path.split(','))
        
        self.directions = self.full_path[self.path_index]
        if len(self.directions) > 0:
            self.pub.publish(self.directions[self.dir_index])
            self.dir_index +=1
        print(self.full_path)

if __name__ == "__main__":
    rospy.init_node('global_planner', anonymous=True)
    Global_planner()
    rospy.spin()