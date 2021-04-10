#!/usr/bin/env python

import serial as ser
import time


class CommandCode:
    START = 'S'
    END = ' E'
    SPEED_FAST = 'a'
    SPEED_SLOW = 's'
    SPLIT_CHAR = '|'

    COMMAND_WRITE = 'A'
    COMMAND_READ = 'B'

class RobotCommand:
    def __init__(self,_port):
        self.__robot_serial = ser.Serial(_port) #open port
        time.sleep(0.2)
        self.clear_serial()
        time.sleep(0.1)

        self.newest_command = []
        self.prev_cmd_time = 0
        self.latest_cmd_time = 0

    def clear_serial(self,_flgs = 0):
        if _flgs == 0:
            self.__robot_serial.reset_input_buffer()
            self.__robot_serial.reset_output_buffer()
        elif _flgs == 1:
            self.__robot_serial.reset_input_buffer()
        elif _flgs == 2:
            self.__robot_serial.reset_output_buffer()

    def has_new_command(self, update_time=True):
        result = self.latest_cmd_time != self.prev_cmd_time
        if result and update_time:
            self.prev_cmd_time = self.latest_cmd_time
        return result

    def write_command(self,command):
        command = str(command)
        if command[0:1] != CommandCode.START:
            command = CommandCode.START + command
        if command[-1:-2] != CommandCode.END:
            command += CommandCode.END

        self.newest_command = [command]
        self.latest_cmd_time = int (time.time() * 1000)
        self.__robot_serial.write(command.encode('utf-8'))
    
    def get_speed(self):
        self.write_command('S B 00000000000000000000007 E')
        return self.__robot_serial.read(6)

    def get_encoder(self):
        self.write_command('S B 00000000000000000000009 E')
        return self.__robot_serial.read(6)
    
    def set_speed(self,data):
        command = str(CommandCode.COMMAND_WRITE)
        for speed in data:
            command += CommandCode.SPLIT_CHAR + str(speed)
        self.write_command(command)

    def set_stop(self,attempt_try = 3):
        for _ in range(attempt_try):
            self.set_speed([0,0,0,0])
            time.sleep(0.02)
    
    def set_max_speed(self, speed):
        self.set_speed([speed, speed, speed, speed])

    
    def set_spin(self, angle, speed = 0.5):
        '''
        Spin robot

        :param angle: angle to spin

        :param speed: speed to spin, default is 0.5 km/h
        '''
        command = [str(CommandCode.COMMAND_SPIN), str(angle), str(speed)]
        command = CommandCode.COMMAND_DELIMITER.join(command)

        self.__write_command(command)
    
    