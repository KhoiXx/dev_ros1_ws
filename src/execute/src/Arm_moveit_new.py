#!/usr/bin/env python

import traceback
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import * 
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Int32, String, Bool, Float32
from move_base_msgs.msg import MoveBaseActionResult
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
import time
import numpy as np
from Robot import Robot
from robot_control import ROBOT_STATUS
from transforms3d.euler import quat2euler, euler2quat

from pynput.keyboard import Key, Listener
from key_mapping import Key_mapping
import datetime
import traceback
import tf2_ros

ROS_WS = '/home/khoixx/dev_ros1_ws'
LOG_FILE_PATH = ROS_WS + '/log/arm_log/'
GOAL_STATUS_SUCCESS = 3
GOAL_STATUS_ABORT = 4

class Fiducials_id:
    PACKAGE_1 = [11,21,31,41,51,61,71,81,91]
    PACKAGE_2 = [12,22,32,42,52,62,72,82,92]
    PACKAGE_3 = [13,23,33,43,53,63,73,83,93]
    shelf_pose = [1,2,3] 
PACKAGE_WIDTH_MIN = 10
PACKAGE_WIDTH_MAX = 45

class moveit_handle():
    def __init__(self):
        self.key_pressed = None
        log_info = [
            LOG_FILE_PATH,
            datetime.datetime.now().strftime('arm_real %d%m_%H:%M'),
            '.log'
        ]
        self.__file_log = open("".join(log_info), "w")
        self.__init_arm()
        self.__init_variables()
        self.__init_topic()
        self.__robot_base = Robot()
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        self.log('Finish initializing arm_moveit')
        rospy.loginfo('Finish initializing arm_moveit {}'.format(self.group.get_end_effector_link()))

    def __init_topic(self):
        rospy.Subscriber('target_position', Pose, self.set_pose_callback)
        rospy.Subscriber('/sonar/package_onboard', Bool, self.package_onboard_callback)
        rospy.Subscriber('/sonar/back_obstacle', Bool, self.back_obstacle_callback)
        # rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.goal_result_callback)
        rospy.Subscriber('/result', Int32, self.result_callback)
        rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, self.aruco_transform)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        find_pose_pub = rospy.Publisher('/finding_pose', Float32, queue_size=10)
        self.gripper_state = rospy.Publisher('/gripper_state', Int32, queue_size=10)
        self.set_rotate_angle_pub = rospy.Publisher('/set_rotate_angle', Int32, queue_size=10)
        # rospy.Timer(rospy.Duration(1), callback = self.show_pose)
    
    # def show_pose(self, timer):
    #     pose = self.group.get_current_pose()
    #     end_link = self.group.get_end_effector_link()
    #     rpy = self.group.get_current_rpy()
    #     log_msg = "current_pose:{} end_link:{} rpy:{}".format(pose, end_link, rpy)
    #     rospy.loginfo(log_msg)
    #     self.log(log_msg)

    def __init_variables(self):
        self.target = Pose()
        self.chosen_id = [0]
        self.package_id = 10
        self.shelf_id = 1
        self.__is_package_onboard = False
        self.__is_back_obstacle = False
        self.__is_package_detect = False
        self.__is_shelf_detect = False
        self.__chosen_id_time = rospy.get_rostime()
        self.__shelf_id_time = rospy.get_rostime()
        self.__joint_state_target = None
        self.scan_range = LaserScan()

    def __init_arm(self):
        self.robot = moveit_commander.RobotCommander()
        self.scence = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander('robotarm')
        self.group.set_named_target("home")
        self.handle_command(1)

    def aruco_transform(self, msg):
        for i in msg.transforms:
            if i.fiducial_id == self.shelf_id:
                self.log("detect shelf")
                self.__shelf_id_time = rospy.get_rostime()
            if i.fiducial_id in self.chosen_id:
                self.__chosen_id_time = rospy.get_rostime()
                self.package_id = i.fiducial_id
                self.log("detect_id")
        self.__is_package_detect = True if rospy.get_rostime().nsecs - self.__chosen_id_time.nsecs < 5*(10**7) else False
        self.__is_shelf_detect = True if rospy.get_rostime().nsecs - self.__shelf_id_time.nsecs < 5*(10**7) else False
        self.log("detect shelf:{} id:{}".format(self.__is_shelf_detect, self.__is_package_detect))

    def set_pose_callback(self,msg):
        # target.position.x = 0.15
        # target.position.y = 0.0275537972778
        # target.position.z = 0.0956095295963
        cp = self.group.get_current_pose()
        rospy.loginfo("Callback: {0}".format(cp))
        self.target.position.x = msg.position.x
        self.target.position.y = msg.position.y
        self.target.position.z = msg.position.z
        self.target.orientation.w = 1.0
        # self.target = self.group.get_random_pose()

        rospy.loginfo('Current position {0} current_target {1}'.format(self.group.get_current_pose(), [self.target]))
        self.log("Position received")

        time.sleep(1)
        self.handle_command(2)

    def goal_result_callback(self, result_msg):
        '''
        Check goal status and correct robot heading
        '''
        try:
            rospy.loginfo("Goal result {0}".format(result_msg.status))
            self.__robot.log("Goal result {0}".format(result_msg.status.status))
            if result_msg.status.status == GOAL_STATUS_SUCCESS:
                if not self.handle_keyboard():
                    return
        except:
            self.log("goal_result_callback@Exception",traceback.format_exc())
    
    ###test fake result goal signal
    def result_callback(self, result_msg):
        '''
        Check goal status and correct robot heading
        '''
        try:
            rospy.loginfo("Goal result")
            self.log("Goal result")
            # if result_msg.data == GOAL_STATUS_SUCCESS:
            #     if not self.handle_keyboard():
            #         return
            # data = input("Nhap position: ")
            # value = data.split(",")
            # if value[0] in ["find_package","find_onshelf","find_onboard","load", "ready"]:
            #     self.group.set_named_target(value[0])
            #     self.handle_command(1)
            #     return
            self.group.set_named_target("ready")
            self.handle_command(1)
            time.sleep(3)
            trans,rot = self.lookup_transform("fiducial_10", "dummy")
            # for i in range(3):
            #     value[i] = float(value[i])
            # value[0] -= 0.06
            # value[1] -= 0.015
            # value[2] += 0.03
            # #link_52 x: 0.128, y 0.036, z: 0.172 r : -1.047, p: 0.018, y: -1.546
            # # self.target.position.x = value[0]
            # # self.target.position.y = value[1]
            # # self.target.position.z = value[2]
            # # self.target.orientation.w = 1.0
            trans.x -= 0.08
            trans.y += 0.048
            trans.z += 0.02
            # self.group.set_position_target(value)
            # self.handle_command(1)
            # trans_x = value[0] - 0.095
            # trans_y = value[1] - 0.032
            rospy.loginfo("position = {}".format([trans.x, trans.y, trans.z]))
            yaw = np.arctan2(trans.y, trans.x) - 1.495
            pose = [trans.x, trans.y, trans.z] +[0.0, 0.0, yaw]
            # pose = self.group.get_current_pose()
            # r,p,y = quat2euler([pose.pose.orientation.w, pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z])
            # r = 0.02
            # p = 0.1
            # goal =[pose.pose.position.x , pose.pose.position.y, pose.pose.position.z, r, p, y]
            rospy.loginfo("Target position: {}, Current_pose:{}".format(pose, self.group.get_current_pose()))

            self.group.set_pose_target(pose)
            a = self.group.plan()
            while not a.joint_trajectory.points:
                rospy.loginfo("plannig")
                pose[5] += 0.02
                self.group.set_pose_target(pose)
                a = self.group.plan()
                if abs(pose[5] - yaw) > 0.5:
                    break
            rospy.loginfo("plan finish {}".format(pose))
            self.handle_command(1)
            self.group.shift_pose_target(0, 0.05)
            
            # joints = self.group.get_current_joint_values()
            # rospy.loginfo(joints)
            # joints[4] = 0.01
            # # self.group.set_joint_value_target(joints)
            # # self.handle_command(1)
            # rpy = self.group.get_current_rpy()
            # self.group.shift_pose_target(3, -rpy[0])
            # rospy.loginfo(rpy)
            # self.handle_command(1)
            # rpy = self.group.get_current_rpy()
            # self.group.shift_pose_target(4, -rpy[1])
            # rospy.loginfo(rpy)
            self.handle_command(1)
            gripper = self.calc_gripper_angle(38)
            self.gripper_state.publish(gripper)
            
        except:
            self.log("result_callback@Exception",traceback.format_exc())
              
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

    def back_obstacle_callback(self, msg):
        ''' save back_obstacle msg publised from sonar'''
        self.__is_back_obstacle = msg.data
        
    def package_onboard_callback(self,msg):
        self.__is_package_onboard = msg.data

    def load_package(self):
        '''Control the arm to load package onto the robot'''
        self.group.set_named_target("find_package")
        self.handle_command(1)
        timeout = 50
        time = rospy.get_rostime()
        delta_t = 5
        speed = 0.14
        while not self.__is_shelf_detect:
            self.__robot_base.set_speed([speed, speed])
            time.sleep(0.2)
            ##check is there any obstacle
            while (self.__is_back_obstacle and speed < 0) or (min(self.scan_range) > 0.4 and speed >0 ):
                self.__robot_base.set_stop()
                if rospy.get_time() - time > timeout:   return False
            if rospy.get_time() - time >= delta_t:
                self.__robot_base.set_stop()
                speed = -speed
                delta_t += 5
            if rospy.get_time() - time > timeout:   return False
        target_frame = '/fiducial_' + str(self.shelf_id)
        time = rospy.get_rostime()
        while rospy.get_rostime() - time < 10:
            translation, rotation = self.lookup_transform(target_frame, 'dummy')
            if abs(translation.x) < 0.02 or translation == None:
                break
            direction = 1 if translation.x > 0 else -1
            self.__robot_base.set_speed([speed*direction, speed*direction])
            time.sleep(0.1)
        self.__robot_base.set_stop()
        if translation.x >=0.02 or translation == None:
            self.log("Cannot get to the shelf")
            rospy.loginfo("Cannot get to the shelf")
            return False
        self.__robot_base.set_rotate(-0.3)
        while self.__robot_base.robot_status != ROBOT_STATUS._STOP:
            current_joints = self.group.get_current_joint_values()
            _,rotarm = self.lookup_transform(target_frame,'link_1')
            _,rotbase = self.lookup_transform('dummy','link_1')
            roll, pitch, yawarm = quat2euler([rotarm.w, rotarm.x, rotarm.y, rotarm.z])
            roll, pitch, yawbase = quat2euler([rotbase.w, rotbase.x, rotbase.y, rotbase.z])
            alpha = -90+yawarm
            current_joints[0] += np.deg2rad(alpha)
            if abs(alpha) > 2 and current_joints[0] <= np.deg2rad(260):
                self.group.set_joint_value_target(current_joints)
                self.handle_command(1)
            time.sleep(0.1)
            if abs(yawbase) < 3:
                self.__robot_base.set_stop()
        ##finished align with shelf
        while min(self.scan_range) >0.27:
            target_frame = "/fiducial_"+self.package_id
            trans,rot = self.lookup_transform(target_frame, "dummy")
            if trans.x > 0.02:
                self.__robot_base.set_speed([0.14,0.17])
            elif trans.x > 0.02:
                self.__robot_base.set_speed([0.17,0.14])
            else:
                self.__robot_base.set_speed([0.14,0.14])
            time.sleep(0.1)
        time.sleep(1)            
        self.__robot_base.set_stop()
        ##stand in front of the package
        
        

    def unload_package(self):
        self.__robot_base.set_stop()
    
    def lookup_transform(self, target_frame, source_frame):
        for i in range(5):
            try:    
                trans = self.tfBuffer.lookup_transform(source_frame,target_frame, rospy.Time()) #lookup_transform(frame dich, frame goc, timeout)
                translation = trans.transform.translation
                rotation = trans.transform.rotation
                break
            except:
                self.log("No transform")
        else:
            return None, None
        return translation, rotation

    def pick_up(self, target_frame):
        self.group.set_named_target("find_onshelf")
        self.handle_command(1)
        translation, rotation = self.lookup_transform(target_frame, 'dummy')
        msg = Int32()
        msg.data = 6
        self.gripper_state.publish(msg)
        self.group.set_position_target(translation)

    def find_shelf(self):
        msg = Int32()
        delta = 10
        while not self.__is_shelf_detect:
            msg.data = delta
            self.set_rotate_angle_pub.publish(msg)
            time.sleep(abs(delta)*0.08)
            if delta == -80:
                msg.data = 80
                self.set_rotate_angle_pub.publish(msg)
                self.log("Couldn't find the shelf")
                rospy.loginfo("Couldn't find the shelf")
                return False
            delta = -delta - 10 if delta >= 0 else -delta + 10
        return True

    def handle_keyboard(self):
        for attemp in range(10):
            try:
                package = input("Choose your package id: ")
                action_to_do = input("Choose your action (load/unload): ")
                if not "load" in action_to_do:
                    continue
                shelf = input("Choose shelf id: ")
                if not package and not shelf:
                    continue
                self.chosen_id = [int(package)] if package else Fiducials_id.PACKAGE_3 if shelf == 3 else Fiducials_id.PACKAGE_2 if shelf == 2 else Fiducials_id.PACKAGE_1
                shelf = int(shelf)  if shelf else 3 if package in Fiducials_id.PACKAGE_3 else 2 if package in Fiducials_id.PACKAGE_2 else 1
                break
            except :
                rospy.logwarn(traceback.format_exc())
        else:
            return False

        if "unload" in action_to_do:
             if not self.unload_package():
                return False
        else:
            if not self.load_package():
                return False
        return True

    def handle_command(self, command):
        '''Chose to command for moveit to execute'''
        try:
            #self.group.set_position_target([self.target.position.x,self.target.position.y,self.target.position.z],end_effector_link=end_link)
            #self.targer.orientation = self.group.get_random_orientation()
            self.log('command: %f',command)
            if command == 0:
                self.group.stop()
                self.group.clear_pose_targets()
                time.sleep(0.5)
            elif command == 1:
                self.log("Executing to joint_target")
                self.group.set_goal_tolerance(0.01)
                self.group.go(wait=True)
                self.group.stop()
                self.group.clear_pose_targets()
                self.log("Finished executing to joint_target")
            elif command == 2:
                self.group.set_goal_tolerance(0.015)
                end_link = self.group.get_end_effector_link()
                self.group.set_pose_target(self.target, end_link)
                self.log("Executing to pose_target")
                self.group.go(wait=True)
                self.group.stop()
                time.sleep(3)
                self.group.clear_pose_targets()
                self.log("Finished executing to pose_target")
            else:
                return False
        except:
            self.log('handle_command@Exception', traceback.format_exc())

    def calc_gripper_angle(self, package_width, rad = False):
        '''This is specified for my robot only
            =====> calculate servo angle suitable for each package width
            my equation: y = -1.09 * x + 60.3 
                         y: gripper angle (deg)
                         x: width of the package (mm)'''
        joint_angle = -1.09 * package_width +60.3
        joint_angle_rad = np.deg2rad(joint_angle)
        if joint_angle_rad <= 0.15:
            joint_angle_rad = 0.15
        elif joint_angle_rad >= 0.9:
            joint_angle_rad = 0.9
        if not rad:
            return np.rad2deg(joint_angle_rad)
        else:
            return joint_angle_rad

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
            


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('Arm_moveit', anonymous = True)
    moveit = moveit_handle()
    # moveit.handle_keyboard()
    # if not moveit.press_handle():
    rospy.spin()
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()

        
