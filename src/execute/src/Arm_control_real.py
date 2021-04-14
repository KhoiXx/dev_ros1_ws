#!/usr/bin/env python3

import sys
import rospy
import roslib
from sensor_msgs.msg import JointState
from adafruit_servokit import ServoKit
import board
import busio
import adafruit_pca9685
import time, os, subprocess, signal
import datetime
from math import pi
import numpy as np

ROS_WS = '/home/khoixx/dev_ros1_ws'
LOG_FILE_PATH = ROS_WS + '/log/arm_log'

J5_ZERO = 0
J4_ZERO = 91
J3_ZERO = 57
J2_ZERO = 100
J1_ZERO = 92
J0_ZERO = 83
position_zero = [J0_ZERO, J1_ZERO, J2_ZERO, J3_ZERO, J4_ZERO, J5_ZERO]

class adjust_joint_angle:
    def __init__(self):
        
        log_info = [
            LOG_FILE_PATH,
            datetime.datetime.now().strftime('arm_real_%d%m_%H%M'),
            '.log'
        ]
        self.__file_log = open("".join(log_info), "w")
        self.name = []
        self.j5_angle = np.rad2deg(-0.00993) + J5_ZERO
        self.j4_angle = np.rad2deg(1.55) + J4_ZERO
        self.j3_angle = np.rad2deg(1.56998) + J3_ZERO
        self.j2_angle = -np.rad2deg(0.80674) + J2_ZERO
        self.j1_angle = np.rad2deg(-1.22291) + J1_ZERO
        self.j0_angle = np.rad2deg(-0.785) + J0_ZERO

        self.gripper_release = True
        self.position = [self.j0_angle,self.j1_angle,self.j2_angle,self.j3_angle,self.j4_angle,self.j5_angle]
        self.pre_position = position_zero
        

        self.test = ServoKit(channels = 16)
        self.log("PCA is opening")
        self.exc_joint = [self.test.servo[5], self.test.servo[4], self.test.servo[3], self.test.servo[2], self.test.servo[1], self.test.servo[0]]
        self.exc_joint[0].actuation_range = 262
        for i in range(0,5):
            self.exc_joint[i].angle = self.fix_angle(self.position[i])
        self.log('Finished initializing arm_control')
        rospy.Subscriber('joint_states', JointState, self.callback)

    def callback(self,msg):
        self.pre_position = self.position
        self.log('pre_position {0}, current_position {1}'.format(self.pre_position, self.pre_position))
        position = msg.position
        position = np.rad2deg(position)
        velocity = msg.velocity
        for i in [0,1,3,4]:
            self.position[i] = position[i] + position_zero[i]
        self.position[2] = -position[2] + J2_ZERO

        self.log("arm is running")
        for i in range(0,4):
            if i == 0:
                if self.position[i] > 260:
                    self.position[i] = 260
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
            
    
    def log(self, *arg):
        '''
        logging to file.
        '''
        # log(self.root_node, arg)
        now = int (time.time() * 1000) # milliseconds
        msg = []
        msg.append(str(now))
        for x in arg:
            msg.append("[" + str(x) + "]")
        msg.append("\n")
        # self.root_node.get_logger().info("".join(msg))
        self.__file_log.write(" ".join(msg))
        




def joint_states_listener():
    rospy.init_node('joint_states_current')
    Arm_Control = adjust_joint_angle()
    rospy.spin()

if __name__ == '__main__':
   joint_states_listener()

    
