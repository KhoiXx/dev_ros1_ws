#!/usr/bin/env python

import serial as ser
import time
import struct
import rospy
import traceback

class CommandCode:
    HEADER = b'\xaa\xff'
    END = b'\x0f\x0e'
    COMMAND_SEND_SPEED = b'\xA0'
    COMMAND_SEND_ENCODER  = b'\xA1'
    COMMAND_SET_SPEED = b'\xA2'
    COMMAND_SET_POSITION = b'\xA3'
    RESP_ACK_SET_POSITION = b'\xB3'
    COMMAND_SPIN = b'\xA4'
    COMMAND_STOP = b'\xA5'
    SET_PID = b'\xB6'

class Ack_response:
    RESP_ACK_SEND_SPEED = b'\xB0'
    RESP_ACK_SEND_ENCODER  = b'\xB1'
    RESP_ACK_SET_SPEED = b'\xB2'
    RESP_ACK_SET_POSITION = b'\xB3'
    RESP_ACK_SPIN = b'\xB4'
    RESP_ACK_STOP = b'\xB5'
    NACK = b'\x4e'
    YACK = b'\x59'

class RobotCommand(object):
    def __init__(self,_port):
        try:
            self.__robot_serial = ser.Serial(_port, baudrate=115200, timeout=0.1) #open port for STM
            if self.__robot_serial.isOpen():
                rospy.loginfo("Port {0} is open".format(_port))
            time.sleep(0.2)
            self.clear_serial()
            time.sleep(0.1)
            self.__speed_data = None
            count = 0
            self.command = CommandCode.COMMAND_STOP
            self.newest_command = []
            self.prev_cmd_time = 0
            self.latest_cmd_time = 0
            self.wh_speed = [0, 0, 0, 0]
            self.wh_encoder = [0, 0, 0, 0]
        except:
            rospy.loginfo("Cannot open serial port @Exception")

    def clear_serial(self,_flgs = 0):
        '''
        flgs = 0: all
        flgs = 1: input
        flgs = 2: output
        '''
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
        command_send = bytearray(command)
        if command_send[0:2] != CommandCode.HEADER:
            command_send = bytearray(CommandCode.HEADER + command_send)
        crc = 0
        for i in range(len(command_send)):
            # crc += int.from_bytes(command_send[i:i+1], 'little')
            crc += struct.unpack("<B", command_send[i:i+1])[0]
        # command_send += crc.to_bytes(2, 'big')
        command_send += struct.pack(">H", crc)
        if command_send[-2:] != CommandCode.END:
            command_send += CommandCode.END
        
        self.newest_command = [command_send]
        self.latest_cmd_time = int (time.time() * 1000)
        
        self.clear_serial(2)
        self.__robot_serial.write(command_send)
        self.__robot_serial.flush()
        # status = self.check_frame(8)
        # if not status:
        #     rospy.loginfo("Not ACK")
        #     self.__robot_serial.write(command_send)
        #     self.__robot_serial.flush()

        # self.clear_serial(1)
        # count += 1
        # rospy.loginfo("count: {0}".format(count))
        # for i in range(5):
        #     status = self.check_frame(8)
        #     if status == True:
        #         break
        #     else:
        #         self.__robot_serial.write(command_send)
        #         continue
        
    def get_imu(self):
        '''
        get data from imu
        return value[[3], [3], [3]]:
        ---------------------------
        [0]: 1: acc_x, acc_y, acc_z
        [1]: 1: roll, pitch, yaw
        [2]: 1: lin_acc_x, lin_acc_y, lin_acc_z
        '''

    
    def get_speed(self):
        '''
        get velocity from base (STM) caculated by encoder
        return value[4]:
        ---------------
        wh_speed[4] fl, fr, br, bl
        '''
        self.command = CommandCode.COMMAND_SEND_SPEED
        self.clear_serial()
        self.write_command(self.command)
        # rospy.loginfo("send get speed")
        self.check_frame(15)
        if not self.__speed_data is None:
            # rospy.loginfo(str(self.__speed_data))
            for i in [0,2,4,6]:
                # self.wh_speed[i] = int.from_bytes(read_data[ i+2 : i+4], 'little')
                self.wh_speed[int(0.5 * i)] = struct.unpack("<h", self.__speed_data[ i+3 : i+5])[0] 
                self.wh_speed[int(0.5 * i)] /= 1000.000
        # rospy.loginfo(self.wh_speed)
        # for i in range(4):
        #     self.wh_speed[i] /= 1000
        return self.wh_speed

    def get_encoder(self):
        '''
        get encoder from base (STM)
        return value[4]:
        ---------------
        wh_encoder[4] fl, fr, br, bl
        '''
        self.command = CommandCode.COMMAND_SEND_ENCODER
        self.write_command(self.command)
        return self.wh_encoder
        
    
    def set_speed(self,data):
        self.command = CommandCode.COMMAND_SET_SPEED
        command = self.command
        for speed in data:
            speed *= 10000
            speed = int(speed)
            if speed <0:
                sign = -1
            else:
                sign = 0
            speed = abs(speed)
            # command += speed.to_bytes(2, 'little')
            command += struct.pack("<Hb", speed, sign)
        self.write_command(command)

    def set_stop(self,attempt_try = 3):
        for _ in range(attempt_try):
            self.command = CommandCode.COMMAND_STOP
            self.write_command(self.command)

    
    def set_spin(self, angle, speed = 0.5):
        self.command = CommandCode.COMMAND_SPIN
        command = self.command
        # command += angle.to_bytes(2, 'little')
        # command += speed.to_bytes(2, 'little')
        command += struct.pack("<H", angle)
        command += struct.pack("<H", speed)
        self.write_command(command)
    
    def check_frame(self, byte_read):
        try:
                
            read_data = self.__robot_serial.read_until(CommandCode.END, byte_read)
            if read_data[0:2] != CommandCode.HEADER or read_data[-2:] != CommandCode.END:
                return False

            if read_data[2:3] == CommandCode.COMMAND_SEND_SPEED:
                if len(read_data) == 15:
                    self.__speed_data = read_data
                    return True
            crc=0

            for i in range(len(read_data) - 4):
                # crc += int.from_bytes(read_data[i: i + 1], 'big')
                crc += struct.unpack(">b", read_data[i:i+1])[0]
            # if int.from_bytes(read_data[-4:-2], 'little') != crc:
            if struct.unpack("<h", read_data[-4:-2])[0] != crc:
                return False
            if read_data[2:3] in Ack_response and byte_read == 8:
                status = False
                if read_data[3:4] == Ack_response.YACK:
                    status = True
                else:
                    status = False
            elif read_data[2:3] in CommandCode and byte_read == 15:
                status = False
                if read_data[2:3] == CommandCode.COMMAND_SEND_SPEED:
                    for i in range(1,4):
                        # self.wh_speed[i] = int.from_bytes(read_data[ i+2 : i+4], 'little')
                        self.wh_speed[i] = struct.unpack("<h", read_data[ i+2 : i+4])[0]
                    status = True
                elif read_data[2:3] == CommandCode.COMMAND_SEND_ENCODER:
                    for i in range(1,4):
                        # self.wh_encoder[i] = int.from_bytes(read_data[ i+2 : i+4], 'little')
                        self.wh_encoder[i] = struct.unpack("<h", read_data[ i+2 : i+4])[0]
                    status = True
                else:
                    status = False
            else:
                return False
            return status
        except:
            return False
                