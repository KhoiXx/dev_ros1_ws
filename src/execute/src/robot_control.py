#!/usr/bin/env python

import serial
import time, os, subprocess, signal
import datetime
import threading
import sys
import math
import numpy as np

from enum import Enum

# from mr001.UtilitiesMacroAndConstant import *
# from mr001.speed_control import SpeedControl
# from distutils.util import strtobool
from robot_command import RobotCommand

ROBOT_WIDTH = 0.26
ROBOT_MAX_SPEED = 0.4
ROBOT_STATUS = Enum('ROBOT_STATUS','_STOP _ROTATING _RUNNING')


NAVIGATE_MODE = 1
KEYBOARD_MODE = 2

ROS_WS = '/home/khoixx/dev_ros1_ws'
LOG_FILE_PATH = ROS_WS + '/log/'

class RobotControl(object):
    def __init__(self, port, baudrate):

        self.MAP_FILE_PATH = ROS_WS + "/map/"
        self.MAP_FILE_PGM = self.MAP_FILE_PATH + "map.pgm"
        self.MAP_FILE_YAML = self.MAP_FILE_PATH + "map.yaml"
        
        self.__current_mode = 0
        self.robot_status = ROBOT_STATUS._STOP
        self.pre_robot_status = ROBOT_STATUS._STOP

        # self.root_node = node_log
        try:

            # "robot_port0,robot_port1,imu_port,lidar_port"
            # self.root_node.get_logger().info('RobotControl@__init__')

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


        # self.speed_control = SpeedControl(self.root_node)


    #def init


    def __init_navigation(self):
        self.path_finding = None
        # start point and target point received from nfc tag
        self.nav_start_point = [] # (x, y, base_yaw_to_map)
        self.nav_target_point = [] # (x, y, base_yaw_to_map)
        # navigation status running
        self.nav_running = False
        self.nav_force_stop = False
        self.nav_start_estimate_point = []
        self.nav_goal_estimate_point = []
    
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

    def get_encoder(self):
        return self.__robot_serial.get_encoder()
    
    def get_speed(self):
        return self.__robot_serial.get_speed()

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
            #self.log_console('Saving map...')
            # backup old map
            self.rename_file(self.MAP_FILE_YAML, self.MAP_FILE_YAML + suffix)  # rename old map "map.yaml" to "map.yaml_Ymd_HMS" for backup
            self.rename_file(self.MAP_FILE_PGM, self.MAP_FILE_PGM + suffix)    # rename old map "map.pgm" to "map.pgm_Ymd_HMS" for backup
            # Run the cmd in a subprocess
            # subprocess.call(['gnome-terminal', '-x', 'rosrun map_server map_saver -f /home/khoixx/dev_ros1_ws/map/map map:=/map'])
            subprocess.call(['gnome-terminal','sh', '--working-directory=/home/khoixx/dev_ros1_ws/map','--command= rosrun map_server map_saver map:=/map'])
            # Wait for map to be saved
            time.sleep(timeout_map)
            # Check if new map files is already created
            save_map_succeed = self.check_file_exist(self.MAP_FILE_PGM, self.MAP_FILE_YAML)
            if save_map_succeed:
                return True

            # failed to create map
            # Kill the subprocess after all

        finally:
            if not save_map_succeed:
                # if save map failed
                # revert old map to its name
                self.rename_file(self.MAP_FILE_YAML + suffix, self.MAP_FILE_YAML)
                self.rename_file(self.MAP_FILE_PGM + suffix, self.MAP_FILE_PGM)

        return False

    def turn_angle(self,_angle,_speed):
        if _speed <0:
            _speed = -_speed
        if _speed >1.6 :
            _speed = 1.6
        elif _speed > 0 and _speed <0.2:
            _speed = 0.2
        
        current_omega = (_speed/ROBOT_WIDTH)*200 #rad/s
        _angle = np.deg2rad(_angle)
        time_reach_angle = _angle/current_omega
        if  _angle < 0:
            _speed = -_speed
      
        while time_reach_angle >0:
            tic = time.clock()
            self.set_speed([_speed,-_speed])
            time_reach_angle -= time.clock() - tic
                
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
    
    def set_navigate_mode(self):
        self.__current_mode = NAVIGATE_MODE

    def is_navigate_mode(self):
        return self.__current_mode == NAVIGATE_MODE
    
    def set_keyboard_mode(self):
        self.__current_mode = KEYBOARD_MODE
    
    def is_keyboard_mode(self):
        return self.__current_mode == KEYBOARD_MODE

    def set_status_stop(self):
        self.robot_status = ROBOT_STATUS._STOP

    def set_status_running(self):
        self.robot_status = ROBOT_STATUS._RUNNING

    def set_status_rotating(self):
        self.robot_status = ROBOT_STATUS._ROTATING

    def is_status_stop(self):
        return self.robot_status == ROBOT_STATUS._STOP
    
    def is_status_rotating(self):
        return self.robot_status == ROBOT_STATUS._ROTATING
    
    def is_status_running(self):
        return self.robot_status == ROBOT_STATUS._RUNNING

    def get_latest_command(self):
        return self.__robot_serial.newest_command

    def log_latest_command(self):
        if self.__robot_serial.has_new_command():
            command = ' '.join(str(self.__robot_serial.newest_command))
            self.log("log_latest_command", command)

    def set_speed(self, speed_data):
        '''
        [vvl,vvr]
        '''
        self.__robot_serial.set_speed(speed_data)
        self.set_status_running()
    
    def set_stop(self,attempt_try = 3):
        self.__robot_serial.set_stop(attempt_try)
        self.set_status_stop()
    
    def set_spin(self, angle, speed = 0.5):
        self.__robot_serial.turn_angle(angle, speed)
        self.set_status_rotating()
    
    def set_rotate(self, speed_rotate):
        '''
        [vvl,vvr]
        speed_rotate: rad/s
        '''
        speed_linear = speed_rotate * ROBOT_WIDTH / 2 # V= W*R
        speed_data = [-speed_linear, speed_linear]
        self.__robot_serial.set_speed(speed_data)
        self.set_status_rotating()

    def time_sleep_to_count(self, time_sleep, timeout):
        return int(timeout / time_sleep)

    def log_running_status(self):
        status = 'ROBOT STATUS:'
        status += ' Rotating[' + str(self.is_status_rotating()) + ']'
        status += ' Running[' + str(self.is_status_running()) + ']'
        status += ' Stop[' + str(self.is_status_stop()) + ']'
        self.log(status)

    def log_running_mode(self):
        status = 'ROBOT RUNNING MODE:'
        status += ' Navigate[' + str(self.is_navigate_mode()) + ']'
        status += ' Keyboard[' + str(self.is_keyboard_mode()) + ']'
        self.log(status)
