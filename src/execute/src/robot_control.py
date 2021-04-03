import serial
import time, os, subprocess, signal
import datetime
import threading
import sys
# from mr001.UtilitiesMacroAndConstant import *
# from mr001.speed_control import SpeedControl
# from distutils.util import strtobool
from robot_command import RobotCommand
import numpy as np

ROBOT_STATUS_STOP = 0
ROBOT_STATUS_ROTATING = 1
ROBOT_STATUS_RUNNING = 2

ROS_WS = '/home/khoixx/dev_ros1_ws'
LOG_FILE_PATH = ROS_WS + '/log/'

class RobotControl:
    def __init__(self, port, baud_rate, node_log):

        self.MAP_FILE_PATH = ROS_WS + "/map/"
        self.MAP_FILE_PGM = self.MAP_FILE_PATH + "map.pgm"
        self.MAP_FILE_YAML = self.MAP_FILE_PATH + "map.yaml"

        self.root_node = node_log
        try:

            # "robot_port0,robot_port1,imu_port,lidar_port"
            self.root_node.get_logger().info('RobotControl@__init__')

            log_info = [
                LOG_FILE_PATH,
                datetime.datetime.now().strftime('%Y%m%d_%H%M%S'),
                '.log'
            ]
            self.__file_log = open("".join(log_info), "w")

            self.log("Opening serial port: " + port + ".")
            self.__robot_serial = RobotCommand(port)

        except serial.serialutil.SerialException as exp:
            self.log("Serial not found at port " + port + ".")
            sys.exit(str(exp))

        # robot at initial state not NAVIGATE_MODE nor LINETRACE_MODE
        self.__current_mode = 0
        self.__robot_status = ROBOT_STATUS_STOP

        # self.speed_control = SpeedControl(self.root_node)

        # for navigation
        self.__init_navigation()

    #def init

    #def else
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
