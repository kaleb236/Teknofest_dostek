#!/usr/bin/env python3
#-*-coding: utf-8 -*-

import tty
import sys
import termios
import rospy

from std_msgs.msg import String

def main():
    rospy.init_node('direction', anonymous=True)
    direction = String()
    pub = rospy.Publisher('qr_state', String, queue_size=10)

    orig_settings = termios.tcgetattr(sys.stdin)

    tty.setcbreak(sys.stdin)
    x = 0
    while x != chr(27): # ESCqqqqqq
        x=sys.stdin.read(1)[0]
        print("You pressed", x)
        direction.data = x
        pub.publish(direction)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)  

if __name__ == '__main__':
    main()