#!/usr/bin/env python

import serial
import time, os, subprocess, signal
import datetime
import threading
import sys
import math

# from mr001.UtilitiesMacroAndConstant import *
# from mr001.speed_control import SpeedControl
# from distutils.util import strtobool
from robot_command import RobotCommand
import numpy as np
from UtilitiesMacroAndConstant import MR_WIDTH

ROBOT_STATUS_STOP = 0
ROBOT_STATUS_ROTATING = 1
ROBOT_STATUS_RUNNING = 2


NAVIGATE_MODE = 1
KEYBOARD_MODE = 2

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

    def flush_data_serial(self, _flg=0):
        self.__robot_serial.clear_serial(_flg)

    def get_encoder_driving(self):
        return self.__robot_serial.get_encoder()

    def set_speed(self, speed_data):
        '''
        set speed to robot

        :param speed_data: [front_left_speed, front_right_speed, rear_left_speed, rear_right_speed]
        '''

        self.__robot_serial.set_speed(speed_data)

    def log_console(self, *arg):
        '''
        logging to console
        '''
        # log(self.root_node, arg)
        now = int (time.time() * 1000) # milliseconds
        msg = []
        msg.append(str(now))
        for x in arg:
            msg.append("[" + str(x) + "]")
        msg.append("\n")
        self.root_node.get_logger().info("".join(msg))

    def check_file_exist(self, path1, path2=None):
        '''
        Check file exists in system

        Parameters:
        -----------
        path1 : str
            file 1
        path2 : str
            file 2
        
        Returns:
        ---------
        bool
            True if file exists
        '''

        if path2:
            return os.path.exists(path1) & os.path.exists(path2)
        return os.path.exists(path1)

    def rename_file(self, old_name, new_name):
        '''
        Rename file

        Parameters:
        -----------
        old_name : str
            file to rename
        
        new_name : str
            new file name
        '''
        if self.check_file_exist(old_name):
            os.rename(old_name, new_name)    

    def save_map(self, timeout_map=15):
        '''
        Save map. http://wiki.ros.org/map_server

        Parameters:
        -----------
        timeout_map : float
            timeout when save map, in seconds
        
        Returns:
        --------
        bool
            True if save map succeed
        '''

        save_map_succeed = False
        suffix = datetime.datetime.now().strftime('_%Y%m%d_%H%M%S')

        try:
            self.log('Saving map...')
            self.log_console('Saving map...')
            # backup old map
            self.rename_file(self.MAP_FILE_YAML, self.MAP_FILE_YAML + suffix)  # rename old map "map.yaml" to "map.yaml_Ymd_HMS" for backup
            self.rename_file(self.MAP_FILE_PGM, self.MAP_FILE_PGM + suffix)    # rename old map "map.pgm" to "map.pgm_Ymd_HMS" for backup

            # Init terminal command to save new map files with name = map.*
            # this command create 2 files: "map.pgm" and "map.yaml"
            # cmd = "ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/map/map"
            map_file_name = "map" # "map.pgm", "map.yaml"
            cmd = "ros2 run nav2_map_server map_saver_cli -f {0}{1}"
            cmd = cmd.format(self.MAP_FILE_PATH, map_file_name)
            # Run the cmd in a subprocess
            subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
            # Wait for map to be saved
            time.sleep(timeout_map)
            # Check if new map files is already created
            save_map_succeed = self.check_file_exist(self.MAP_FILE_PGM, self.MAP_FILE_YAML)
            if save_map_succeed:
                return True

            # failed to create map
            # Kill the subprocess after all
            id_proc = int(subprocess.check_output(["pidof", "map_saver"]))
            os.kill(id_proc, signal.SIGKILL)

        finally:
            if not save_map_succeed:
                # if save map failed
                # revert old map to its name
                self.rename_file(self.MAP_FILE_YAML + suffix, self.MAP_FILE_YAML)
                self.rename_file(self.MAP_FILE_PGM + suffix, self.MAP_FILE_PGM)

        return False

    def release_motor(self, param=''):
        '''
        Release motor

        :param param: 0 for release 4 wheels, 1 for release 4 steering, otherwise release all
        '''
        self.__robot_serial.set_stop()

    def turn_angle(self,_angle,_speed):
        if _speed >1.6 :
            _speed = 1.6
        elif _speed > 0 and _speed <0.2:
            _speed = 0.2
        if _speed <0:
            _speed = -_speed
        
        current_omega = (_speed/MR_WIDTH)*200 #rad/s
        _angle = np.deg2rad(_angle)
        time_reach_angle = _angle/current_omega
        if  _angle < 0:
            _speed = -_speed
      
        while time_reach_angle >0:
            tic = time.clock()
            self.set_speed([_speed,-_speed,_speed,-_speed])
            time_reach_angle -= time.clock()-tic
                
    def correct_angle(self, angle, is_radian=True):
        '''
        Correcting angle make sure angle between [-180 ~ 180]

        Parameters:
        -----------
        angle : float
            angle need to correcting
        
        is_radian : bool, optional
            True if angle is in radian or else False
        '''

        if not is_radian:
            # convert to radian
            angle = np.deg2rad(angle)

        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi
        
        if not is_radian:
            # convert back to degree
            angle = np.rad2deg(angle)

        return angle
    
    def set_keyboard_mode(self):
        self.__current_mode = KEYBOARD_MODE
    
    def is_keyboard_mode(self):
        return self.__current_mode == KEYBOARD_MODE