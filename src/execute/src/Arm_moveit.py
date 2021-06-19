#!/usr/bin/env python

import traceback
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import *
from std_msgs.msg import Int32, String, Bool, Float32
from move_base_msgs.msg import MoveBaseActionResult
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
import time
import numpy as np
from Robot import Robot
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
    PACKAGE_1 = 10
    PACKAGE_2 = 11
    PACKAGE_3 = 12
    LOADING_POINT = 1
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
        # rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.goal_result_callback)
        rospy.Subscriber('/result', Int32, self.result_callback)
        rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, self.aruco_transform)

        # rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        find_pose_pub = rospy.Publisher('/finding_pose', Float32, queue_size=10)
        self.set_rotate_angle_pub = rospy.Publisher('/set_rotate_angle', Int32, queue_size=4)

    def __init_variables(self):
        self.target = Pose()
        self.chosen_id = 0
        self.__is_package_onboard = False
        self.__is_package_detect = False
        self.transform = Transform()
        self.__is_shelf_detect = False
        self.__chosen_id_time = rospy.get_rostime()
        self.__shelf_id_time = rospy.get_rostime()
        self.__joint_state_target = None

    def __init_arm(self):
        self.robot = moveit_commander.RobotCommander()
        self.scence = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander('robotarm')
        self.group.set_named_target("home")
        self.group.go()
        self.group.stop()
        time.sleep(3)
        self.group.clear_pose_targets()
        self.handle_command(2)

    def aruco_transform(self, msg):
        for i in msg.transforms:
            if i.fiducial_id == Fiducials_id.LOADING_POINT:
                self.__shelf_id_time = rospy.get_rostime()
            if self.chosen_id:
                if i.fiducial_id == self.chosen_id:
                    self.__chosen_id_time = rospy.get_rostime()
        self.__is_package_detect = True if rospy.get_rostime().nsecs - self.__chosen_id_time.nsecs < 5*(10**7) else False
        self.__is_shelf_detect = True if rospy.get_rostime().nsecs - self.__shelf_id_time.nsecs < 5*(10**7) else False

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
        self.group.set_goal_tolerance(0.01)

        rospy.loginfo('Current position {0} current_target {1}'.format(self.group.get_current_pose(), [self.target]))
        self.log("Position received")

        time.sleep(1)
        self.handle_command(2)

    def handle_keyboard(self):
        package = input("Choose your package id: ")
        return package

    def handle_command(self, command):
        '''Chose to command for moveit to execute'''
        try:
            end_link = self.group.get_end_effector_link()
            #self.group.set_position_target([self.target.position.x,self.target.position.y,self.target.position.z],end_effector_link=end_link)
            #self.targer.orientation = self.group.get_random_orientation()
            self.group.set_pose_target(self.target, end_link)
            self.log('command: %f',command)
            if command == 0:
                self.group.stop()
                time.sleep(0.5)
            elif command == 1:
                rospy.loginfo("is planning")
                self.group.plan()
                time.sleep(5)
                rospy.loginfo("plan finished")
            elif command == 2:
                
                rospy.loginfo("is executing")
                self.group.go(wait=True)
                self.group.stop()
                time.sleep(3)
                self.group.clear_pose_targets()
                rospy.loginfo("execute finished")
            else:
                return False
        except:
            self.log('handle_command@Exception', traceback.format_exc())

    def package_onboard_callback(self,msg):
        try:
            self.__is_package_onboard = msg.data
        except:
            self.__robot.log("package_onboard_callback@Exception", traceback.format_exc())
    
    def goal_result_callback(self, result_msg):
        '''
        Check goal status and correct robot heading
        '''
        try:
            rospy.loginfo("Goal result {0}".format(result_msg.status))
            self.__robot.log("Goal result {0}".format(result_msg.status.status))
            if result_msg.status.status == GOAL_STATUS_SUCCESS:
                if not self.__is_package_onboard:
                    self.load_package()
                elif self.__is_package_onboard:
                    self.unload_package()
        except:
            self.__robot.log("goal_result_callback@Exception",traceback.format_exc())
    
    def result_callback(self, result_msg):
        '''
        Check goal status and correct robot heading
        '''
        try:
            rospy.loginfo("Goal result")
            self.log("Goal result")
            if result_msg.data == GOAL_STATUS_SUCCESS:
                if not self.__is_package_onboard:
                    self.load_package()
                elif self.__is_package_onboard:
                    self.unload_package()
        except:
            self.__robot.log("result_callback@Exception",traceback.format_exc())

    def load_package(self):
        '''Control the arm to load package onto the robot'''
        self.chosen_id = int(self.handle_keyboard())
        self.group.set_named_target("find_onself")
        self.group.go(wait=True)
        self.group.stop()
        joints= self.group.get_current_joint_values()
        #move robot close to shelf (distance from shelf 0.3 ~ 0.32)
        in_position = False
        while self.find_shelf():
            target_frame = 'fiducial_1' #the loading point aruco
            try:    
                trans = self.tfBuffer.lookup_transform(target_frame, 'dummy', rospy.Time()) #lookup_transform(frame dich, frame goc, timeout)
                translation = trans.transform.translation
                rotation = trans.transform.rotation
                if translation.x <= 0.30:
                    self.__robot_base.set_speed([-0.14, -0.14])
                elif translation.x >= 0.32:
                    self.__robot_base.set_speed([0.14, 0.14])
                else:
                    self.__robot_base.set_stop()
                    in_position = True
                    break
                time.sleep(0.4)
                self.__robot_base.set_stop()
            except :
                self.log("No transform")
        if not in_position: return
        msg = Int32()
        roll, pitch, yaw = quat2euler([rotation.w, rotation.x, rotation.y, rotation.z])
        msg.data = int(180 + np.rad2deg(yaw))
        self.set_rotate_angle_pub.publish(msg)
        joints[0].position = np.deg2rad(130)
        self.group.set_joint_value_target(joints)
        self.group.go(wait=True)
        self.group.stop()

    def unload_package(self):
        self.__robot_base.set_stop()
    
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

    def calc_gripper_angle(self, package_width, rad = True):
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

        
