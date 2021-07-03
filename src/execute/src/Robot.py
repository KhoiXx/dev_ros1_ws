#import section
import serial as ser
import os
import numpy as numpy
import sys
import rospy
import traceback
import time

from key_mapping import Key_mapping
from serial import Serial
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int8

from key_mapping import Key_mapping
from robot_control import RobotControl, ROBOT_WIDTH
from Vehicle import Vehicle

#variables

SPEED_MIN_VALUE = 0.4
DIRECTION_FORWARD = 1
DIRECTION_BACKWARD = -1
#main code
PORT = '/dev/ttyTHS1'
class Robot(RobotControl):
    def __init__(self,__port = PORT,__baudrate = 115200):

        super(Robot, self).__init__(__port, __baudrate)
        self.log("The robot is connected to ", __port)
        self._current_speed = SPEED_MIN_VALUE
        self.direction = 1
        self.target_goal = PoseStamped()
        self.initial_topic()
        # self.log_console("Robot finished initialize...")

    def initial_topic(self):
        self.log("Initial robot's topic...")
        self.sub_key = rospy.Subscriber('keyboard_control', String, self.keycontrol_callback)
        self.status_pub = rospy.Publisher('robot_status', String, queue_size=10)
        rospy.Timer(rospy.Duration(0.02), callback = self.robot_status_pub) #50Hz

    def robot_status_pub(self, timer):
        pre_status = self.pre_robot_status
        status = str(self.robot_status)
        if status != pre_status :
            self.status_pub.publish(status)
        self.pre_robot_status = status
    
    def keycontrol_callback(self, msg):
        # self.log_latest_command()
        if not self.is_keyboard_mode():
            self.set_keyboard_mode()
            # reset speed
            self._current_speed = SPEED_MIN_VALUE

        command = msg.data
        # # Show log
        # self.log("Data from: [" + str(command) + ']')
        self.set_status_running()
        self.handle_command(command)

    def handle_command(self, command):
        '''
        Handle command received from rimocon
        '''

        try:
            key_command = str(command)
            command = ''
            if key_command == Key_mapping.FAST:
                self._current_speed = 0.2
                return
            
            if key_command == Key_mapping.SLOW:
                self._current_speed = 0.15
                return

            if key_command == Key_mapping.ROTATE_LEFT:
                if not self._current_speed:
                    self._current_speed = 0.6
                speed_rotate = self._current_speed *0.8 / (ROBOT_WIDTH / 2)

                self.set_rotate(speed_rotate)
                return

            if key_command == Key_mapping.ROTATE_RIGHT:
                if not self._current_speed:
                    self._current_speed = 0.6
                speed_rotate = - self._current_speed *0.8 / (ROBOT_WIDTH / 2)

                self.set_rotate(speed_rotate)
                return

            if key_command == Key_mapping.LEFT:
                if not self._current_speed:
                    self._current_speed = 0.6
                self.set_speed([self.direction * self._current_speed * 0.4, self.direction * self._current_speed])
                return

            if key_command == Key_mapping.RIGHT:
                if not self._current_speed:
                    self._current_speed = 0.6
                self.set_speed([self.direction * self._current_speed, self.direction * self._current_speed * 0.4])
                return

            if key_command == Key_mapping.STOP:
                self.set_stop()
                return

            if key_command == Key_mapping.FORWARD:
                if not self._current_speed:
                    self._current_speed = 0.6
                self.direction = DIRECTION_FORWARD
                self.set_speed([self._current_speed, self._current_speed])
                return

            if key_command == Key_mapping.BACK:
                if not self._current_speed:
                    self._current_speed = 0.6
                self.direction = DIRECTION_BACKWARD
                self.set_speed([-self._current_speed, -self._current_speed])
                return
                
            # if key_command == Key_mapping.COMMAND_SAVE_MAP:
            #     if self.save_map(timeout_map=15):
            #         self.log("Save map successfully...")
            #         # self.log_console("Save map successfully...")
            #     else:
            #         self.log("Save map failed...")
            #         # self.log_console("Save map failed...")
            #     return

            self.log("handle_key_command INVALID command [" + key_command + "]")
        except Exception as ex:
            error_msg = traceback.format_exc()
            self.log("handle_key_command@Exception", error_msg)
