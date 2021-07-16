#!/usr/bin/env python3

import sys
import rospy
import roslib
from sensor_msgs.msg import JointState
from adafruit_servokit import ServoKit
from std_msgs.msg import Int32
import board
import busio
import adafruit_pca9685
import time, os, subprocess, signal
import datetime
from math import pi
import numpy as np

ROS_WS = '/home/khoixx/dev_ros1_ws'
LOG_FILE_PATH = ROS_WS + '/log/arm_log/'

J5_ZERO = 0
J4_ZERO = 178
J3_ZERO = 149
J2_ZERO = 73
J1_ZERO = 34
J0_ZERO = 219
position_zero = [J0_ZERO, J1_ZERO, J2_ZERO, J3_ZERO, J4_ZERO, J5_ZERO]

class adjust_joint_angle:
    def __init__(self):
        self.name = []
        self.j5_angle = J5_ZERO
        self.j4_angle = np.rad2deg(0.1) + J4_ZERO
        self.j3_angle = J3_ZERO
        self.j2_angle = J2_ZERO
        self.j1_angle = J1_ZERO
        self.j0_angle = J0_ZERO

        self.gripper_release = True
        self.position = [self.j0_angle,self.j1_angle,self.j2_angle,self.j3_angle,self.j4_angle,self.j5_angle]
        self.pre_position = position_zero
        

        self.test = ServoKit(channels = 16)
        for i in range(6):
            self.test.servo[i].set_pulse_width_range(min_pulse = 370, max_pulse = 2340)
        # self.log("PCA is opening")
        self.exc_joint = [self.test.servo[5], self.test.servo[4], self.test.servo[3], self.test.servo[2], self.test.servo[1], self.test.servo[0]]
        self.exc_joint[0].actuation_range = 260
        for i in range(6):
            self.exc_joint[i].angle = self.fix_angle(self.position[i])
        # self.log('Finished initializing arm_control')
        rospy.Subscriber('joint_states', JointState, self.callback)
        rospy.Subscriber('gripper_state', Int32, self.gripper_callback)

    def callback(self,msg):
        self.pre_position = self.position
        # self.log('pre_position {0}, current_position {1}'.format(self.pre_position, self.pre_position))
        position = msg.position
        position = np.rad2deg(position)
        velocity = msg.velocity
        for i in [0,1,3,4]:
            self.position[i] = position[i] + position_zero[i]
        self.position[2] = -position[2] + J2_ZERO

        # self.log("arm is running")
        for i in range(5):
            if i == 0:
                if self.position[i] > 270:
                    self.position[i] = 270
                elif self.position[i] < 0:
                    self.position[i] = 0
                self.exc_joint[i].angle = self.position[i]
            else:
                self.exc_joint[i].angle = self.fix_angle(self.position[i])
        time.sleep(0.01)

        # if self.gripper_release:
        #     self.exc_joint[0].angle = 0
        # else:
        #     self.exc_joint[0].angle = 10
        
        # rospy.loginfo("Current Position: %f",self.position)
    def gripper_callback(self, msg):
        angle = msg.data
        angle = 5 if angle < 5 else 53 if angle > 53 else angle
        i = (angle - float(self.exc_joint[5].angle))
        for value in range(abs(int(i))):
            self.exc_joint[5].angle += 1 if i >0 else -1
            time.sleep(0.07)


    def set_angle(self,_joint):
        i = self.pre_position[_joint]
        while i != self.position[_joint]:
            if self.position[_joint] > self.pre_position[_joint]:
                i += 1
            elif self.position[_joint] < self.position[_joint]:
                i -= 1
            else:
                break
            self.log("i position: {0}_{1}".format(_joint, i))
            self.exc_joint[_joint].angle = self.pre_position[_joint] + i
            time.sleep(0.07)
    
    def fix_angle(self,_angle):
        if _angle >180:
            _angle = 180
        elif _angle <0:
            _angle = 0
        return _angle

def joint_states_listener():
    rospy.init_node('joint_states_current')
    Arm_Control = adjust_joint_angle()
    rospy.spin()

if __name__ == '__main__':
   joint_states_listener()

    
