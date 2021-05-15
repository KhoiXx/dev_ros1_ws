#!/usr/bin/env python

import os
import sys
import rospy
import tf
import threading
import numpy as np
import traceback

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, PoseStamped, PoseWithCovarianceStamped, Vector3
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
        self.previous_calib_time = rospy.Time.now()
        self.__init_topic()
        self.__init_flag()
        rospy.loginfo("Initialization navigation finished")
        self.__robot.log("Initialization navigation finished")

        self.imu_data = [[0.00, 0.00, 0.00], 
                         [0.00, 0.00, 0.00], 
                         [0.00, 0.00, 0.00]]
        
        self.velocity = [0.00, 0.00, 0.00] #vx, vy, vth
        self.pose =[0.00, 0.00, 0.00] #x, y, th
    
    def __init_topic(self):
        self.__robot.log("Start initializing navigation topic")
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        #update odom
        rospy.Timer(rospy.Duration(0.02), callback = self.odom_update) #50Hz
        self.odom_raw_pub = rospy.Publisher('/odom', Odometry, queue_size=20)
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
        Subscription to imu/data topic (published by imu_filter_node
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

    def odom_update(self, timer):
        '''
        update odom with data from sensor
        '''
        self.__robot.log("Update odom")
        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()
        v_straight = self.__robot.get_speed()
        v_left = (v_straight[0] + v_straight[3]) / 2
        v_right = (v_straight[1] + v_straight[2]) / 2

        v_avg= (v_left + v_right) / 2
        # delta_s = v_avg * dt
        
        # yaw_angle = self.imu_data[0][2]
        # yaw_angle = self.__correct_angle(yaw_angle)

        self.velocity[0] = self.imu_data[1][0] #vx
        self.velocity[1] = self.imu_data[1][1] #vy
        self.velocity[2] = self.imu_data[2][2] #vth

        vy = self.velocity[0]
        vx = self.velocity[1]
        vth = self.velocity[2]
        
        # compute odometry in a typical way given the velocities of the robot
        delta_x = (vx * np.cos(self.pose[2]) - vy * np.sin(self.pose[2])) * dt
        delta_y = (vx * np.sin(self.pose[2]) + vy * np.cos(self.pose[2])) * dt
        delta_th = vth * dt
        self.__robot.log_running_status()
        log_msg = "pre_delta_X:{0} pre_delta_y:{1} status:{2}".format(delta_x, delta_y, self.__robot.robot_status)
        self.__robot.log(log_msg)

        if self.__robot.is_status_stop() or self.__robot.is_status_rotating():
            delta_x = 0
            delta_y = 0

        log_msg = "delta_X:{0} delta_y:{1} status:{2}".format(delta_x, delta_y, self.__robot.is_status_stop())
        self.pose[0] += delta_x
        self.pose[1] += delta_y
        self.pose[2] += delta_th
        #log_msg = "vx: {0} vy:{1} vth:{2} posex:{3} posey:{4} posez:{5} vstraight: {5}".format(vx,vy,vth, self.pose[0], self.pose[1], self.pose[2], v_straight)
        self.__robot.log(log_msg)
        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.pose[2])

        # first, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.pose[0], self.pose[1], 0.),
            odom_quat,
            self.current_time,
            "base_link",
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.pose[0], self.pose[1], 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # self.__robot.log("Odom data: {0}".format(odom))

        # publish the message
        self.odom_raw_pub.publish(odom)

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

    def __correct_angle(self, angle, is_radian=True):
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

        if angle > np.pi:
            angle -= 2 * np.pi
        elif angle < -np.pi:
            angle += 2 * np.pi
        
        if not is_radian:
            # convert back to degree
            angle = np.rad2deg(angle)

        return angle