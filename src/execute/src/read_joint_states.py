#!/usr/bin/env python3

import sys
import rospy
import roslib
from sensor_msgs.msg import JointState
from adafruit_servokit import ServoKit
import board
import busio
import adafruit_pca9685
import time
from math import pi
import numpy as np


J0_ZERO = 0
J1_ZERO = 1.588
J2_ZERO = 0.995
J3_ZERO = 1.745
J4_ZERO = 1.606
J5_ZERO = 1.4449
position_zero = [J0_ZERO, J1_ZERO, J2_ZERO, J3_ZERO, J4_ZERO, J5_ZERO]

class adjust_joint_angle:
    def __init__(self):
        rospy.Subscriber('joint_states', JointState, self.callback)
        self.name = []
        self.j0_angle = -0.7849 + J0_ZERO
        self.j1_angle = -1.2230 + J1_ZERO
        self.j2_angle = 0.8066 + J2_ZERO
        self.j3_angle = 1.5699 + J3_ZERO
        self.j4_angle = 1.5699 + J4_ZERO
        self.j5_angle = -0.0099 + J5_ZERO
        self.temp = []
        self.position = [self.j0_angle,self.j1_angle,self.j2_angle,self.j3_angle,self.j4_angle,self.j5_angle]
        self.pre_position = position_zero
        try:
            self.test = ServoKit(channels = 16)
            self.exc_joint = [self.test.servo[0], self.test.servo[1], self.test.servo[2], self.test.servo[3], self.test.servo[4], self.test.servo[5]]
            self.exc_joint[5].actuation_range = 262
        except:
            # rospy.loginfo("cannot connect to pca9685")
            pass

    def callback(self,msg):
        self.pre_position = self.position
        position = msg.position
        velocity = msg.velocity
        for i in range(0,5):
            self.position[i] = position[i] + position_zero[i]
        print("ok")
        # rospy.loginfo("Current Position: %f",self.position)
    
    def set_angle(self,_joint,_angle):
       
        for i in range(self.pre_position[_joint],self.position[_joint]):
            i += 1
            self.exc_joint[_joint].angle = self.position[_joint] + i
            time.sleep(0.05)




def joint_states_listener():
    rospy.init_node('joint_states_current')
    adjust_joint_angle()
    rospy.spin()

if __name__ == '__main__':
   joint_states_listener()

    
