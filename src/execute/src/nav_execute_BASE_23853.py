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
from robot_control import ROBOT_WIDTH, ROBOT_MAX_SPEED

class Navigation:
    def __init__(self, _robot):
        _robot = Robot()
        self.__robot = _robot
        rospy.loginfo("Start initializing navigation")
        self.__robot.log("Start initializing navigation")
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.previous_calib_time = rospy.Time.now()
        
        self.velocity = [0.00, 0.00, 0.00] #vx, vy, vth
        self.__init_topic()
        self.__init_flag()
        rospy.loginfo("Initialization navigation finished")
        self.__robot.log("Initialization navigation finished")

        self.imu_data = [[0.00, 0.00, 0.00], 
                         [0.00, 0.00, 0.00], 
                         [0.00, 0.00, 0.00]]
        
        self.pose =[0.00, 0.00, 0.00] #x, y, th
        self.pre_cmd_vel = Twist()
        self.goal_pose = Pose()
        rospy.Timer(rospy.Duration(0.05), callback = self.odom_update) #10Hz
        
    
    def __init_topic(self):
        self.__robot.log("Start initializing navigation topic")
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        #update odom

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
        try:
            # self.__robot.log('cmd_vel_callback', str(cmd_vel_msg))

            linear_x = cmd_vel_msg.linear.x
            angular_z = cmd_vel_msg.angular.z
            self.pre_cmd_vel = cmd_vel_msg
            rospy.loginfo("Receive cmd_vel: {0} {1}".format(linear_x, angular_z))
            if linear_x == 0.0 and angular_z == 0.0:
                self.__robot.set_stop()
                return
            self.calculate_cmd(linear_x, angular_z)
        except:
            self.__robot.log("cmd_vel_callback@Exception", traceback.format_exc())
            

        
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
        self.goal_pose = goal_msg.pose
        goal_roll, goal_pitch, goal_yaw = quat2euler([self.goal_pose.orientation.w, self.goal_pose.orientation.x, self.goal_pose.orientation.y, self.goal_pose.orientation.z])
        self.__robot.log("position: {0}, yaw: {1}".format(self.goal_pose.position, goal_yaw))
        rospy.loginfo("goal_yaw: {}".format(goal_yaw))

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
        # self.__robot.log("Update odom
        # ")
        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()
        v_straight = self.__robot.get_speed()
        # rospy.loginfo(v_straight)
        v_left = (v_straight[0] + v_straight[3]) / 2
        v_right = (v_straight[1] + v_straight[2]) / 2

        v_avg= (v_left + v_right) / 2
        vth = self.imu_data[2][2]
        delta_s = v_avg * dt
        #rospy.loginfo(v_left)
        #rospy.loginfo(v_right)

        yaw_angle = self.imu_data[0][2]
        yaw_angle = self.__correct_angle(yaw_angle)
        
        delta_th = vth * dt
        self.pose[2] += self.__correct_angle(delta_th)


        delta_x = delta_s * np.cos(self.pose[2])
        delta_y = delta_s * np.sin(self.pose[2])
        # rospy.loginfo("delta th{0}".format (self.pose[2]))

        vx = delta_x/dt
        vy = delta_y/dt

        if self.__robot.is_status_stop() or self.__robot.is_status_rotating():
            delta_x = 0
            delta_y = 0

        log_msg = "vl:{0} vr:{1} yaw:{2}".format(v_left, v_right, self.pose[2])
        self.pose[0] += delta_x
        self.pose[1] += delta_y
        
        #log_msg = "vx: {0} vy:{1} vth:{2} posex:{3} posey:{4} posez:{5} vstraight: {5}".format(vx,vy,vth, self.pose[0], self.pose[1], self.pose[2], v_straight)
        self.__robot.log(log_msg)
        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.pose[2])

        # first, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.pose[0], self.pose[1], 0.),
            odom_quat,
            self.current_time,
            "dummy",
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.pose[0], self.pose[1], 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "dummy"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # self.__robot.log("Odom data: {0}".format(odom))

        # publish the message
        self.odom_raw_pub.publish(odom)

        self.last_time = self.current_time
    
    def calculate_cmd(self, linear, angular):
        rospy.loginfo("Handling cmd_vel msg")
        if linear == 0.0:
            # speed_wh = angular*ROBOT_WIDTH / 2 #v = w*r
            self.__robot.set_rotate(self.check_speed(angular, is_angular=True))
            return
        # self.__robot.log("Rotate robot to cmd_vel")
        if angular == 0.0:
            speed = self.check_speed(linear)
            self.__robot.set_speed([speed, speed])
            return
        else:
            direction = 1 if linear  > 0 else -1
            R_c = linear / angular
            R_l = R_c - ROBOT_WIDTH / 2   # r  center left
            R_r = R_c + ROBOT_WIDTH / 2   # r center right
            v_whl = abs(angular * R_l) * direction
            v_whr = abs(angular * R_r) * direction
            v_whl = self.check_speed(v_whl)
            v_whr = self.check_speed(v_whr)
            self.__robot.log("v left: {0} v right: {1}".format(v_whl, v_whr))
            self.__robot.set_speed([v_whl, v_whr])
            return
        
    def check_speed(self, speed, is_angular=False):
        if is_angular:
            speed *= ROBOT_WIDTH / 2 
        if speed > ROBOT_MAX_SPEED:
            speed = ROBOT_MAX_SPEED
        elif speed < -ROBOT_MAX_SPEED:
            speed = -ROBOT_MAX_SPEED
        
        if speed > 0.04 and speed < 0.11:
            speed = 0.11
        elif speed < -0.04 and speed > -0.11:
            speed = -0.11
        if is_angular:
            speed /= (ROBOT_WIDTH / 2)
        return speed

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