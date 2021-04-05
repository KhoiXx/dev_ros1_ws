#!/usr/bin/env python3

import sys
import rospy
import roslib
from sensor_msgs.msg import JointState
from adafruit_servokit import ServoKit
from math import pi
import numpy as np

global position, velocity

position = [0, 0, 0, 0, 0, 0]

class adjust_joint_angle:
    def __init__(self):
        self.name = []
        self.j0_angle = []
        self.j1_angle = []
        self.j2_angle = []
        self.j3_angle = []
        self.j4_angle = []
        self.j5_angle = []
        self.temp = []
        self.rad2deg()
    def rad2deg(self,position):
        for i in range(0,5):
            self.temp[i] = round(position[i]*180/pi,2)





def callback(msg):
    position = msg.position
    velocity = msg.velocity
    rospy.loginfo("Current Position: %s",position)


def joint_states_listener():
    rospy.init_node('joint_states_current')
    rospy.Subscriber('joint_states', JointState, callback)
    rospy.spin()



if __name__ == '__main__':
   joint_states_listener()

    
