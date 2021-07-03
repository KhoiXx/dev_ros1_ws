#!/usr/bin/env python

import os
import sys
import rospy
import tf
import threading
import numpy as np
import traceback
import threading
import time
from enum import Enum

from std_msgs.msg import String, Bool, Float32, Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, PoseStamped, PoseWithCovarianceStamped, Vector3
from transforms3d.euler import quat2euler, euler2quat
from sensor_msgs.msg import Imu, MagneticField, LaserScan
from actionlib_msgs.msg import GoalStatusArray, GoalID
from move_base_msgs.msg import MoveBaseActionResult
from fiducial_msgs.msg import FiducialTransformArray

from Robot import Robot
from robot_control import ROBOT_WIDTH, ROBOT_MAX_SPEED

MAX_YAW_TOLERANCE = 0.15
GOAL_STATUS_SUCCESS = 3
GOAL_STATUS_ABORT = 4

ROBOT_MODE = Enum('ROBOT_MODE','_NORMAL _NAV _RECOVERY _FIND_POSE')

class Navigation:
    def __init__(self):
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
        self.goal_posestamped = PoseStamped()
        self.goal_yaw = 0.0
        self.range_max = None
        self.range_min = None
        self.scan_range = []
        self.v_straight = None
        rospy.Timer(rospy.Duration(0.05), callback = self.odom_update) #10Hz
        
    
    def __init_topic(self):
        self.__robot.log("Start initializing navigation topic")
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.goal_result_callback)
        rospy.Subscriber('/sonar/back_obstacle', Bool, self.back_obstacle_callback)
        # rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/finding_pose', Float32, self.finding_pose_callback)
        rospy.Subscriber('/set_rotate_angle', Int32, self.set_rotate_angle_callback)

        rospy.Subscriber('/test_speed', Float32, self.test_speed)

        #update odom
        self.nav_success_pub = rospy.Publisher('/nav_success', Bool, queue_size=1)
        self.odom_raw_pub = rospy.Publisher('/odom', Odometry, queue_size=20)
        self.cancel_goal_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.__robot.log("Initialization navigation topic finished")

    def __init_flag(self):
        # self.__is_nav_mode = False
        # self.__is_recovery_mode = False
        self.__current_position = None
        self.__current_goal_pose = None
        self.__is_back_obstacle = False
        self.__is_package_onboard = False
        self.__is_turning = False
        self.__is_finding_pose = False
        self.__base_mode = ROBOT_MODE._NORMAL               

    def test_speed(self, speed):
        data = speed.data
        self.__robot.set_speed([data, data])

    def cmd_vel_callback(self, cmd_vel_msg):
        '''
        Subscription to cmd_vel topic (published by ros navigation)
        Params:
        -------
        cmd_vel_msg: Twist
        '''
        
        if self.__base_mode != ROBOT_MODE._NAV:
            rospy.loginfo("Returning")
            self.__robot.log("Returning {0} ".format(self.__base_mode))
            return
        try:
            # self.__robot.log('cmd_vel_callback', str(cmd_vel_msg))
            linear_x = cmd_vel_msg.linear.x
            angular_z = cmd_vel_msg.angular.z
            self.pre_cmd_vel = cmd_vel_msg
            rospy.loginfo("Receive cmd_vel: {0} {1}".format(linear_x, angular_z))
            # if abs(self.v_straight[0] - self.v_straight[3]) > 0.1 or abs(self.v_straight[1] - self.v_straight[2]) > 0.1:
            #     self.__robot.set_stop()
            #     time.sleep(1)
            #     return
            if linear_x == 0.0 and angular_z == 0.0:
                self.__robot.log("cmd stop")
                self.__robot.set_stop()
                return
            self.calculate_cmd(linear_x, angular_z)
            self.__robot.log("speed:{}".format(self.v_straight))
        except:
            self.__robot.log("cmd_vel_callback@Exception", traceback.format_exc())
            
    def scan_callback(self,scan_msg):
        '''
        Subscription to odom topic (published by ros depth_image_to_scan)
        Params:
        -------
        scan_msg: LaserScan
        '''
        temp = self.scan_range
        self.scan_range = []
        index = int(len(scan_msg.ranges)/2)
        for i in scan_msg.ranges[index-200:index+200]:
            if not np.isnan(i):
                self.scan_range  += [i]
        if not self.scan_range:
            self.scan_range = temp
            return
        self.range_max = max(self.scan_range)
        self.range_min = min(self.scan_range)
        # rospy.loginfo("Scan max: {} scan min: {} len:{}".format(max(self.scan_range), min(self.scan_range), len(self.scan_range)))
        
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
        self.__base_mode = ROBOT_MODE._NAV
        self.goal_pose = goal_msg.pose
        self.goal_posestamped = goal_msg
        goal_roll, goal_pitch, self.goal_yaw = quat2euler([self.goal_pose.orientation.w, self.goal_pose.orientation.x, self.goal_pose.orientation.y, self.goal_pose.orientation.z])
        self.__robot.log("position: {0}, yaw: {1}".format(self.goal_pose.position, self.goal_yaw))
        rospy.loginfo("goal_yaw: {}".format(self.goal_yaw))

    def goal_result_callback(self, result_msg):
        '''
        Check goal status and correct robot heading
        '''
        try:
            rospy.loginfo("Goal result {0}".format(result_msg.status))
            self.__robot.log("Goal result {0}".format(result_msg.status.status))
            if result_msg.status.status == GOAL_STATUS_SUCCESS:                
                self.correct_robot_heading()
            # elif result_msg.status.status == GOAL_STATUS_ABORT:
            #     #this mean robot stuck ====> try to recover and set goal again
            #     self.run_recovery()
            #     return
        except:
            self.__robot.log("goal_result_callback@Exception",traceback.format_exc())

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

    def back_obstacle_callback(self, msg):
        ''' save back_obstacle msg publised from sonar'''
        try:
            self.__is_back_obstacle = msg.data
        except:
            self.__robot.log("back_obstacle_callback@Exception", traceback.format_exc())
    
    def finding_pose_callback(self, msg):
        '''
        Base control to finding the pose to pick up package 
        Params:
        -------------
        msg: Float32
        '''
        self.__base_mode = ROBOT_MODE._FIND_POSE
        self.__is_finding_pose = True
        speed = self.check_speed(msg.data)
        self.__robot.log("Finding_pose_callback msg:{0}".format(msg.data))
        if not speed:
            self.__robot.set_stop()
            time.sleep(1)
            self.__robot.log("Finding_pose_callback: set_stop success")
        else:
            self.__robot.set_speed([speed, speed])
            self.__robot.log("Finding_pose_callback: set_speed {0} success".format(speed))
        
    def set_rotate_angle_callback(self, msg):
        self.__robot.log("rotate testing {0}: yaw:{1}".format(msg.data, self.pose[2]))
        rospy.loginfo("rotate testing {0}: yaw:{1}".format(msg.data, self.pose[2]))
        delta = self.rotate_angle(self.__correct_angle(np.deg2rad(msg.data)), True)
        self.__robot.log("delta: {0}".format(delta))

    def odom_update(self, timer):
        '''
        update odom with data from sensor
        '''
        # self.__robot.log("Update odom
        # ")
        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()
        self.v_straight = self.__robot.get_speed()
        # rospy.loginfo(self.v_straight)
        v_left = (self.v_straight[0] + self.v_straight[3]) / 2
        v_right = (self.v_straight[1] + self.v_straight[2]) / 2

        v_avg= (v_left + v_right) / 2
        vth = self.imu_data[2][2]
        delta_s = v_avg * dt
        #rospy.loginfo(v_left)
        #rospy.loginfo(v_right)
    
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
        
        #log_msg = "vx: {0} vy:{1} vth:{2} posex:{3} posey:{4} posez:{5} vstraight: {5}".format(vx,vy,vth, self.pose[0], self.pose[1], self.pose[2], self.v_straight)
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
        '''caculate and control robot with command from cmd_vel'''
        rospy.loginfo("Handling cmd_vel msg")
        if linear < 0 and self.__is_back_obstacle:
            self.__robot.set_stop()
            msg = GoalID()
            msg.stamp = rospy.Time.now()
            msg.id = ""
            self.cancel_goal_pub.publish(msg)
            self.__robot.log("Cancel goal because obstacle is in the back")
            return
        if linear == 0.0:
            # speed_wh = angular*ROBOT_WIDTH / 2 #v = w*r
            self.__robot.log("cmd rotate")
            self.__robot.set_rotate(self.check_speed(angular, is_angular=True))
            return
        # self.__robot.log("Rotate robot to cmd_vel")
        if angular == 0.0:
            self.__robot.log("cmd forward")
            speed = self.check_speed(linear)
            self.__robot.log("speed:{0}, obstacle:{1}".format(speed, self.__is_back_obstacle))
            self.__robot.set_speed([speed, speed])
            return
        else:
            self.__is_turning = True
            self.__robot.log("cmd turn")
            direction = 1 if linear  > 0 else -1
            R_c = linear / angular
            R_l = R_c - ROBOT_WIDTH / 2   # r  center left
            R_r = R_c + ROBOT_WIDTH / 2   # r center right
            v_whl = abs(angular * R_l) * direction
            v_whr = abs(angular * R_r) * direction
            ratio = float(v_whl/v_whr)
            if max(ratio, 1/ratio) <= 0.25/0.14:
                if ratio < 1:
                    v_whl = self.check_speed(speed = v_whl, max_speed=0.2)
                    v_whr = self.check_speed(speed = v_whl/ratio, max_speed=0.5)
                else:   
                    v_whr = self.check_speed(speed = v_whr, max_speed=0.2)
                    v_whl = self.check_speed(speed = v_whr * ratio, max_speed=0.5)
            else:
                if not max(v_whl, v_whr) > 0.14 and not min(v_whr, v_whl) < -0.14:
                    if v_whr >= v_whl >= 0: v_whr = 0.14
                    elif v_whl > v_whr >= 0: v_whl = 0.14
                    elif v_whr <= v_whl <= 0: v_whr = -0.14
                    elif v_whl < v_whr <= 0: v_whl = -0.14

            self.__robot.log("v left: {0} v right: {1}".format(v_whl, v_whr))
            self.__robot.set_speed([v_whl, v_whr])
            self.__is_turning = False
            return
        
    def check_speed(self, speed, is_angular=False, max_speed = ROBOT_MAX_SPEED):
        '''limit speed in valid range'''
        if is_angular:
            speed *= ROBOT_WIDTH / 2 
        if speed > max_speed:
            speed = max_speed
        elif speed < -max_speed:
            speed = -max_speed
        
        if speed > 0.04 and speed < 0.14:
            speed = 0.14
        elif speed < -0.04 and speed > -0.14:
            speed = -0.14
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
        angle %= 2*np.pi
        if angle > np.pi:
            angle -= 2 * np.pi
        elif angle < -np.pi:
            angle += 2 * np.pi
        
        if not is_radian:
            # convert back to degree
            angle = np.rad2deg(angle)

        return angle
    
    def correct_robot_heading(self):
        '''Spin head robot to minimize to tolerance of yaw goal'''
        try:
            self.__robot.log("Navigation succeed ===> Correct robot heading")
            self.__base_mode = ROBOT_MODE._NORMAL
            self.__robot.set_stop()
            time.sleep(1)
             
            if self.rotate_angle(self.goal_yaw) <= 0.05:
                self.__robot.log("Correct heading succeeded")
                self.nav_success_pub.publish(Bool(True))
            else:
                self.__robot.log("Correct heading failed")
        except:
            self.__robot.log("corect_robot_heading@Exception", traceback.format_exc())
    
    def run_recovery(self):
        '''Try to unstuck robot'''
        try:
            self.__robot.log("Navigation failed ===> Recovery")
            self.__base_mode = ROBOT_MODE._RECOVERY          
            self.__robot.log(self.__is_back_obstacle)
            if self.__is_back_obstacle:
                self.__robot.log("Cannot go backward, there is a obstacle")
                rospy.loginfo("Cannot go backward, there is a obstacle. Try to move forward")
                self.__robot.set_speed([0.15, 0.15])
                for i in range (50):
                    if self.range_min >= 0.5: ##bug
                        time.sleep(0.1)
                    else:
                        self.__robot.set_stop()
                        self.__base_mode = ROBOT_MODE.NORMAL 
                        return
            else:
                self.__robot.log("Try to move backward")
                speed = 0.15
                self.__robot.set_speed([-speed, -speed])
                for i in range (50): #keep moving back in 5s
                    if self.__is_back_obstacle:
                        break
                    time.sleep(0.1)
            self.__robot.set_stop()
            self.__robot.log("Try to set goal again")
            self.goal_pub.publish(self.goal_posestamped)
            self.__base_mode = ROBOT_MODE.NORMAL 
        except:
            self.__robot.log("run_recovery@Exception", traceback.format_exc())

    def rotate_angle(self, angle, is_angle = False):
        yaw_now = self.pose[2]
        yaw_target = yaw_now + angle if is_angle else angle
        time_start = rospy.get_rostime().secs
        log_msg = "yaw_now:{} delta:{} ".format(yaw_now, yaw_now-yaw_target)
        # rospy.loginfo(log_msg)
        self.__robot.log(log_msg)
        while abs(yaw_now - yaw_target)>0.05:
            direction = 1 if yaw_now <= yaw_target else -1
            self.__robot.set_rotate(self.check_speed(1.2*direction, is_angular=True))
            time.sleep(0.1)
            if rospy.get_rostime().secs - time_start > 30:
                rospy.loginfo("ok")
                break
            yaw_now = self.pose[2]
            log_msg = "yaw_now:{} delta:{} direction:{}".format(yaw_now, yaw_now-yaw_target, direction)
            rospy.loginfo(log_msg)
            self.__robot.log(log_msg)
            if abs(yaw_now - yaw_target) <=0.05:
                self.__robot.set_stop() 
                time.sleep(2)
                yaw_now = self.pose[2]
        self.__robot.set_stop() 
        return abs(self.pose[2] - yaw_target)
