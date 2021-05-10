#!/usr/bin/env python

import os
import sys
import rospy
import tf
import threading

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, PoseStamped, PoseWithCovarianceStamped
from transforms3d.euler import quat2euler, euler2quat
from sensor_msgs.msg import Imu, MagneticField

from Robot import Robot

class Navigation:
    def __init__(self, _robot):
        _robot = Robot()
        self.__robot = _robot
        rospy.loginfo("Start initializing navigation")
        self.__robot.log("Start initializing navigation")
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.__init_topic()
        self.__init_flag()
        rospy.loginfo("Initialization navigation finished")
        self.__robot.log("Initialization navigation finished")

        self.imu_data = [[0.00, 0.00, 0.00], 
                         [0.00, 0.00, 0.00], 
                         [0.00, 0.00, 0.00]]
    
    def __init_topic(self):
        self.__robot.log("Start initializing navigation topic")
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        #update odom
        rospy.Timer(rospy.Duration(10**8), callback = self.odom_update)
        self.odom_raw_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.__robot.log("Initialization navigation topic finished")

    def __init_flag(self):
        self.__is_nav_mode = False
        self.__is_recoverty_mode = False
        self.__current_position = None
        self.__current_goal_pose = None
    
    def cmd_vel_callback(self, cmd_vel_msg):
        '''
        Subscription to cmd_vel topic (published by ros navigation)
        Params:
        -------
        cmd_vel_msg: Twist
        '''

    def odometry_callback(self, odom_msg):
        '''
        Subscription to odom topic (published by ros navigation)
        Params:
        -------
        odom_msg: Odometry
        '''
    
    def goal_callback(self, goal_msg):
        '''
        Subscription to goal topic (published by rviz)
        Params:
        -------
        goal: PoseStamped
        '''

    def imu_callback(self, imu_msg):
        '''
        Subscription to imu/data topic (published by imu_filter_node)
        Params:
        -------
        imu: Imu
        '''
        try:
            roll, pitch, yaw = quat2euler([imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z])
            self.imu_data = [[roll, pitch, yaw],
                            [imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z],
                            [imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z]]
        except:
            self.__robot.log("imu_callback@Exception", traceback.format_exc())

    def odom_update(self):
        '''
        update frequency 10Hz
        '''
        self.current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()
        
        delta_x = (vx * cos(th) - vy * sin(th)) * dt
        delta_y = (vx * sin(th) + vy * cos(th)) * dt
        delta_th = vth * dt
        self.last_time = self.current_time

    def __get_vel(self):
        '''
        Get current velocity from STM32 (whl & whr)
        '''
        self.__robot.get_speed()
    
    def __get_imu(self):
        '''
        Get imu data from arduino
        '''
