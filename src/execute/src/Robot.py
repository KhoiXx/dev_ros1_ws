#import section
import serial as ser
import os
import numpy as numpy
import sys
import rospy
import traceback

from key_mapping import Key_mapping
from serial import Serial
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from key_mapping import Key_mapping
from robot_control import RobotControl
from UtilitiesMacroAndConstant import *
from Vehicle import Vehicle

#variables
rosserial_port = '/dev/ttyTHS1'

#main code

class Robot(RobotControl):
    def __init__(self,port = PORT,baudrate = BAUDRATE,node_log=None):

        super(Robot, self).__init__(port, baud_rate, node_log)
        self.vehicle = Vehicle(CENTER_X, CENTER_Y, MR_WIDTH / 2)
        self.log("The robot is connected to ", port)
        self._current_speed = 1.4
        self.target_goal = PoseStamped()
        self.initial_topic()
        

    def initial_topic(self):
        '''
        create publisher, subscription for robot
        '''
        self.log("Initial robot's topic...")
        # self.root_node.create_subscription(String, 'nfc_reader', self.nfc_reader_callback, 10)
        self.root_node.create_subscription(String, 'keyboard_control', self.keycontrol_callback, 10)
    
    def keycontrol_callback(self, msg):

        if not self.is_keyboard_mode():
            self.set_keyboard_mode()
            # reset speed
            self._current_speed = 1.4

        command = msg.data
        # Show log
        self.log("Data from: [" + str(command) + ']')]
        self.handle_command(command)

    def handle_command(self, command):
        '''
        Handle command received from rimocon
        '''

        try:
            key_command = str(command)
            command = ''

            if key_command == Key_mapping.ROTATE_LEFT:
                # turn left 90 degree
                # "q,-90,-90,-90,-90"
                if not _current_speed:
                    _current_speed = 1
                self.turn_angle(-90,_current_speed)
                return

            if key_command == Key_mapping.ROTATE_RIGHT:
                # turn right 90 degree
                # "q,90,90,90,90"
                if not _current_speed:
                    _current_speed = 1
                self.turn_angle(90,_current_speed)
                return

            if key_command == Key_mapping.LEFT:
                # rotate-left
                # "3,180;"
                # SPIN at min speed for running slam
                self.rimocon_current_angle = -1
                self.set_spin(Key_mapping.RIMOCON_ANGLE_90 * 20, 0.3)
                return

            if key_command == Key_mapping.RIGHT:
                # rotate-right
                # "3,-180;"
                # SPIN at min speed for running slam
                self.rimocon_current_angle = -1
                self.set_spin(Key_mapping.RIMOCON_ANGLE_90 * 20 * -1, 0.3)
                return

            if key_command == Key_mapping.STOP:
                # stop
                # "8;"
                self.release_motor()
                return

            if key_command == Key_mapping.FORWARD:
                # move forward
                # "0,1000;"
                if self.rimocon_current_angle != 0:
                    self.rimocon_current_angle = 0

                    steering_data = []
                    for _ in range(4):
                        steering_data.append(self.rimocon_current_angle)

                    self.set_steering(steering_data)
                    time.sleep(Key_mapping.DELAY_FOR_STEER_45)

                self.set_move_distance(Key_mapping.RIMOCON_DISTANCE_MOVE)
                return

            if key_command == Key_mapping.BACK:
                # move backward
                # "0,-1000;"
                if self.rimocon_current_angle != 0:
                    self.rimocon_current_angle = 0

                    steering_data = []
                    for _ in range(4):
                        steering_data.append(self.rimocon_current_angle)

                    self.set_steering(steering_data)
                    time.sleep(Key_mapping.DELAY_FOR_STEER_45)

                self.set_move_distance(Key_mapping.RIMOCON_DISTANCE_MOVE * -1)
                return
                
            if key_command == Key_mapping.COMMAND_SAVE_MAP:
                if self.save_map(timeout_map=15):
                    self.log("Save map successfully...")
                    self.log_console("Save map successfully...")
                else:
                    self.log("Save map failed...")
                    self.log_console("Save map failed...")
                return

            self.log("handle_key_command INVALID command [" + key_command + "]")
        except Exception as ex:
            error_msg = traceback.format_exc()
            self.log("handle_key_command@Exception", error_msg)