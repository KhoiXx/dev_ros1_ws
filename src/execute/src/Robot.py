#import section
import serial as ser
import os
import numpy as numpy
import sys
import rospy


from key_mapping import Key_mapping
from serial import Serial
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from key_mapping import Key_mapping
from robot_control import RobotControl

#variables
rosserial_port = '/dev/ttyTHS1'

#main code

class Robot(RobotControl):
    def __init__(self,port = rosserial_port,baudrate = 115200):
        super(Robot, self).__init__(port, baud_rate, node_log)
        self.log("The robot is connected to ", port)
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

        if not self.is_rimocon_mode():
            self.set_rimocon_mode()
            # reset speed
            self.rimocon_current_speed = Key_mapping.SPEED_MIN_VALUE

        command = msg.data
        # Show log
        self.log("Data from rimocon: [" + str(command) + ']')
        # Write command from rimocon to nexus robot
        self.set_status_running()
        self.handle_command(command)

    def handle_command(self, command):
        '''
        Handle command received from rimocon
        '''

        try:
            rimocon_command = str(command)
            command = ''

            if rimocon_command == Key_mapping.LEFT:
                # turn left 90 degree
                # "q,-90,-90,-90,-90"
                self.set_speed()

                return

            if rimocon_command == Key_mapping.RIGHT:
                # turn right 90 degree
                # "q,90,90,90,90"
                if abs(self.rimocon_current_angle) != abs(Key_mapping.RIMOCON_ANGLE_90):
                    self.rimocon_current_angle = Key_mapping.RIMOCON_ANGLE_90

                    steering_data = []
                    for _ in range(4):
                        steering_data.append(self.rimocon_current_angle)

                    self.set_steering(steering_data)
                    time.sleep(Key_mapping.DELAY_FOR_STEER_90)

                self.set_move_distance(Key_mapping.RIMOCON_DISTANCE_MOVE)

                return

            if rimocon_command == Key_mapping.ROTATE_LEFT:
                # rotate-left
                # "3,180;"
                # SPIN at min speed for running slam
                self.rimocon_current_angle = -1
                self.set_spin(Key_mapping.RIMOCON_ANGLE_90 * 20, 0.3)
                return

            if rimocon_command == Key_mapping.ROTATE_RIGHT:
                # rotate-right
                # "3,-180;"
                # SPIN at min speed for running slam
                self.rimocon_current_angle = -1
                self.set_spin(Key_mapping.RIMOCON_ANGLE_90 * 20 * -1, 0.3)
                return

            if rimocon_command == Key_mapping.SPEED_LEVEL_SLOW:
                # slow  0.5 km/h
                # "n,0.5,0.5,0.5,0.5;"
                self.rimocon_current_speed = Key_mapping.SPEED_MIN_VALUE
                self.set_max_speed(self.rimocon_current_speed)
                return

            if rimocon_command == Key_mapping.SPEED_LEVEL_MEDIUM:
                # medium  1.0 km/h
                # "n,1,1,1,1;"
                self.rimocon_current_speed = Key_mapping.SPEED_MEDIUM_VALUE
                self.set_max_speed(self.rimocon_current_speed)
                return

            if rimocon_command == Key_mapping.SPEED_LEVEL_FAST:
                # fast 1.5 km/h
                # "n,1.5,1.5,1.5,1.5;"
                self.rimocon_current_speed = Key_mapping.SPEED_LEVEL_FAST
                self.set_max_speed(self.rimocon_current_speed)
                return

            if rimocon_command == Key_mapping.STOP:
                # stop
                # "8;"
                self.release_motor()
                return

            if rimocon_command == Key_mapping.RED_STOP:
                # stop
                # "8;"
                self.stop_emergency()
                return

            if rimocon_command == Key_mapping.FORWARD:
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

            if rimocon_command == Key_mapping.BACK:
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
                
            if rimocon_command == Key_mapping.COMMAND_SAVE_MAP:
                if self.save_map(timeout_map=15):
                    self.log("Save map successfully...")
                    self.log_console("Save map successfully...")
                else:
                    self.log("Save map failed...")
                    self.log_console("Save map failed...")
                return

            self.log("handle_rimocon_command INVALID command [" + rimocon_command + "]")
        except Exception as ex:
            error_msg = traceback.format_exc()
            self.log("handle_rimocon_command@Exception", error_msg)