#!/usr/bin/env python
import sys
import rospy
import serial
import time, os, subprocess, signal
import datetime
import traceback
import numpy as np

from std_msgs.msg import String, Bool
from sensor_msgs.msg import Imu, MagneticField
from robot_control import ROBOT_STATUS

ROS_WS = '/home/khoixx/dev_ros1_ws'
LOG_FILE_PATH = ROS_WS + '/log/imu/' 

class IMU_Sensor(object):
    def __init__(self, port, baudrate=115200):
        try:
            self.__serial_imu = serial.Serial(port, baudrate, timeout=0.5)
            if self.__serial_imu.isOpen():
                rospy.loginfo("Port {0} is open".format(port))
            self.clear_serial()
            time.sleep(3.0)
        

            self.magX = 0.0
            self.magY = 0.0
            self.magZ = 0.0

            self.accX = 0.0
            self.accY = 0.0
            self.accZ = 0.0

            self.gyroX = 0.0
            self.gyroY = 0.0
            self.gyroZ = 0.0

            self.is_back_obstacle = 0
            self.is_package = 0
            # self.log("Opening serial port: {0}".format(port))
        except:
            sys.exit(traceback.format_exc())


    def clear_serial(self,_flgs = 0):
        if _flgs == 0:
            self.__serial_imu.reset_input_buffer()
            self.__serial_imu.reset_output_buffer()
        elif _flgs == 1:
            self.__serial_imu.reset_input_buffer()
        elif _flgs == 2:
            self.__serial_imu.reset_output_buffer()

    def init_imu(self):
        self.readIMU()
        return [self.magX, self.magY, self.magZ, self.accX, self.accY, self.accZ, self.gyroX, self.gyroY, self.gyroZ]

    def readIMU(self):
        step = 0
        while step < 10:
            step += 1
            # try to read 10 times
            try:
                self.clear_serial(2)
                self.__serial_imu.write('1'.encode())
                raw = str(self.__serial_imu.readline().decode('utf-8')).split(',')
                
                data = [(float(i)) for i in raw]
                
                self.magX = data[0]
                self.magY = data[1]
                self.magZ = data[2]

                self.accX = data[3] # m/s^2
                self.accY = data[4] # m/s^2
                self.accZ = data[5] # m/s^2

                self.gyroX = data[6]
                self.gyroY = data[7]
                self.gyroZ = data[8]

                self.is_back_obstacle = data[9]
                self.is_package = data[10]
                break

            except Exception as exp:
                rospy.loginfo("ReadIMU@Exception")

    def calib_offset(self):
        try:
            self.log("Cablib offset")
            self.__serial_imu.write('2'.encode())
            time.sleep(0.5)
        except:
            rospy.loginfo("calibIMU@Exception")

class IMU_node(IMU_Sensor):
    def __init__(self, _port, _baud):
        try:
            log_info = [
                LOG_FILE_PATH,
                'imu_',
                datetime.datetime.now().strftime('%Y%m%d_%H%M%S'),
                '.log'
            ]
            self.log_to_file = open("".join(log_info),"w")
            super(IMU_node, self).__init__(_port, _baud)
            rospy.loginfo("IMU is ready")
            rospy.Subscriber("robot_status", String, self.read_robot_status)
            self.pub_acc_gyr_raw = rospy.Publisher("/imu/data_raw", Imu, queue_size=20)
            self.pub_mag_raw = rospy.Publisher("/imu/mag", MagneticField, queue_size=10)
            self.pub_back_obstacle = rospy.Publisher("/sonar/back_obstacle", Bool, queue_size=2)
            self.pub_package_onboard = rospy.Publisher("/sonar/package_onboard", Bool, queue_size=2)
            self.initMessage()

            self.__robot_status = ROBOT_STATUS._STOP
            self.__pre_robot_status = ROBOT_STATUS._STOP

            time.sleep(1.5) #wait for imu init
            self.init_data = self.init_imu()
            rospy.Timer(rospy.Duration(0.02), callback = self.publish_imu_raw) #50Hz
            self.__time_now = rospy.Time.now()
            self.__time_pre = rospy.Time.now()

        except:
            rospy.loginfo("Cannot open serial port {0}".format(_port))
            self.log("Cannot open serial port")

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
        self.log_to_file.write(" ".join(msg))

    def initMessage(self):
        self.acc_gyr = Imu()
        self.acc_gyr.header.frame_id = "/imu"
        self.mag = MagneticField()
        self.mag.header.frame_id = "/imu"

    def publish_imu_raw(self, timer):
        try:
            self.readIMU()
            self.__time_now = rospy.Time.now()
            self.acc_gyr.linear_acceleration.x = self.accX 
            self.acc_gyr.linear_acceleration.y = self.accY 
            self.acc_gyr.linear_acceleration.z = self.accZ
            self.acc_gyr.angular_velocity.x = self.gyroX 
            self.acc_gyr.angular_velocity.y = self.gyroY 
            self.acc_gyr.angular_velocity.z = self.gyroZ 
            self.acc_gyr.header.stamp = self.__time_now
            self.pub_acc_gyr_raw.publish(self.acc_gyr)

            self.mag.magnetic_field.x = self.magX
            self.mag.magnetic_field.y = self.magY
            self.mag.magnetic_field.z = self.magZ
            self.mag.header.stamp = self.__time_now
            self.pub_mag_raw.publish(self.mag)
            self.log("accx: {0} accy: {1}".format(self.accX, self.accY))
            
            msg = Bool()
            msg.data = bool(self.is_back_obstacle)
            self.pub_back_obstacle.publish(msg)
            msg.data = bool(self.is_package)
            self.pub_package_onboard.publish(msg)

            # if self.__time_now - self.__time_pre > 5000:
            #     self.calib_offset()
            #     self.__time_pre = self.__time_now

        except:
            rospy.loginfo("Publish IMU fail")
            self.log("Publish IMU fail")
    
    def read_robot_status(self, status_msg):
        self.log("Read robot_status")
        self.__pre_robot_status = self.__robot_status
        self.__robot_status = status_msg.data

def main(_port, _baud):
    try:
        rospy.init_node('imu_node')
        rospy.loginfo("ROS Node IMU_node")
        try:
            IMU_node(_port=_port, _baud=_baud)
            rospy.loginfo("IMU is connected {0}".format(_port))
        except Exception as ex:
            sys.exit(str(ex))
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down IMU node")
        rospy.on_shutdown()

if __name__ == '__main__':
    imu_port = rospy.get_param('~port', '/dev/ttyUSB0')
    baudrate= int(rospy.get_param('~baud','115200'))
    main(imu_port, baudrate)