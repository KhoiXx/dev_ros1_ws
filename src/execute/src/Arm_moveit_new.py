#!/usr/bin/env python

import traceback
import rospy
import sys
import moveit_commander
import time
import numpy as np
import datetime
import traceback
import tf2_ros

from geometry_msgs.msg import *
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Int32, String, Bool, Float32
from move_base_msgs.msg import MoveBaseActionResult
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from transforms3d.euler import quat2euler, euler2quat

from Robot import Robot
from robot_control import ROBOT_STATUS

ROS_WS = "/home/khoixx/dev_ros1_ws"
LOG_FILE_PATH = ROS_WS + "/log/arm_log/"
GOAL_STATUS_SUCCESS = 3
GOAL_STATUS_ABORT = 4


class Fiducials_id:
    PACKAGE_1 = [10, 20, 30, 40, 50]
    PACKAGE_2 = [11, 21, 31, 41, 51]
    PACKAGE_3 = [12, 22, 32, 42, 52]
    shelf_pose = [1, 2, 3]


PACKAGE_WIDTH_MIN = 10
PACKAGE_WIDTH_MAX = 45


def shelf_position(id="A1"):
    case = {
        "A1": Point(2.112, -0.467, 0.00),
        "A2": Point(2.12, -0.643, 0.00),
        "A3": Point(2.144, -0.771, 0.00),
        "B1": Point(3.34, 1.25, 0.00),
        "B2": Point(3.43, 1.25, 0.00),
        "B3": Point(3.52, 1.25, 0.00),
    }
    return case.get(id, Point(2.292, -0.467, 0.00))


def shelf_orientation(id="A1"):
    case = {
        # "A1": Quaternion(0.00, 0.00, -0.0169992, 0.9998555),
        # "A2": Quaternion(0.00, 0.00, -0.0169992, 0.9998555),
        # "A3": Quaternion(0.00, 0.00, -0.0169992, 0.9998555),
        "A1": Quaternion(0.00, 0.00, -0.00, 1),
        "A2": Quaternion(0.00, 0.00, -0.00, 1),
        "A3": Quaternion(0.00, 0.00, -0.00, 1),
        "B1": Quaternion(0.00, 0.00, 0.6997161, 0.714421),
        "B2": Quaternion(0.00, 0.00, 0.7142137, 0.6999277),
        "B3": Quaternion(0.00, 0.00, 0.7173561, 0.6967067),
    }
    return case.get(id, Quaternion(0.00, 0.00, 0.0169992, 0.9998555))


class moveit_handle:
    def __init__(self):
        log_info = [
            LOG_FILE_PATH,
            datetime.datetime.now().strftime("arm_real %d%m_%H:%M"),
            ".log",
        ]
        self.__file_log = open("".join(log_info), "w")
        self.__init_variables()
        self.__init_topic()
        self.__init_arm()
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        self.log("Finish initializing arm_moveit")
        self.speed_pub.publish(Float32(0.0))
        rospy.loginfo(
            "Finish initializing arm_moveit {}".format(
                self.group.get_end_effector_link()
            )
        )

    def __init_topic(self):
        ## Subcribe
        # rospy.Subscriber("target_position", Pose, self.set_pose_callback)
        rospy.Subscriber("/mytopic/back_obstacle", Bool, self.back_obstacle_callback)
        rospy.Subscriber("/mytopic/nav_success", Bool, self.goal_result_callback)
        rospy.Subscriber("/mytopic/result", Int32, self.result_callback)
        rospy.Subscriber(
            "/fiducial_transforms", FiducialTransformArray, self.aruco_transform
        )
        rospy.Subscriber("/mytopic/yaw_angle", Float32, self.yaw_angle_callback)
        ##Publish
        # find_pose_pub = rospy.Publisher("/finding_pose", Float32, queue_size=10)
        self.gripper_state = rospy.Publisher("/gripper_state", Int32, queue_size=10)
        self.set_rotate_angle_pub = rospy.Publisher(
            "/mytopic/set_rotate_angle", Int32, queue_size=10
        )
        self.set_move_distance_pub = rospy.Publisher(
            "/mytopic/set_move_distance", Float32, queue_size=10
        )
        # rospy.Timer(rospy.Duration(1), callback = self.show_pose)
        self.goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=10
        )
        self.speed_pub = rospy.Publisher("/mytopic/test_speed", Float32, queue_size=10)

    def __init_variables(self):
        self.target = Pose()
        self.chosen_id = 10
        self.package_width = 36
        self.package_id = 12
        self.shelf_id = 2
        self.__is_back_obstacle = False
        self.__is_package_detect = False
        self.__is_shelf_detect = False
        self.__chosen_id_time = rospy.get_rostime()
        self.__shelf_id_time = rospy.get_rostime()
        self.scan_range = LaserScan()
        self.package_count = 0
        self.goal_shelf = PoseStamped()
        self.goal_shelf.header.frame_id = "map"
        self.action_to_do = "unload"
        self.detect_id = ""
        self.count = 0
        self.yaw_angle_deg = 0

    def __init_arm(self):
        self.robot = moveit_commander.RobotCommander()
        self.scence = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("robotarm")
        self.group.set_named_target("home")
        self.handle_command(1)
        self.calc_gripper_angle()

    def aruco_transform(self, msg):
        """
        Handle msg from aruco_detect
        Params:
        ==========
        msg: FiducialTransformArray
        """
        for i in msg.transforms:
            if i.fiducial_id == self.shelf_id:
                # self.log("detect shelf")
                self.__shelf_id_time = rospy.get_rostime()
            if i.fiducial_id == self.chosen_id:
                self.__chosen_id_time = rospy.get_rostime()
                self.package_id = i.fiducial_id
                # self.log("detect_id")
            self.detect_id = "fiducial_" + str(i.fiducial_id)
            if i.fiducial_id in Fiducials_id.PACKAGE_1:
                self.package_width = 36
            elif i.fiducial_id in Fiducials_id.PACKAGE_2:
                self.package_width = 20
        self.__is_package_detect = (
            True
            if rospy.get_rostime().nsecs - self.__chosen_id_time.nsecs < 5 * (10 ** 7)
            else False
        )
        self.__is_shelf_detect = (
            True
            if rospy.get_rostime().nsecs - self.__shelf_id_time.nsecs < 5 * (10 ** 7)
            else False
        )
        # self.log("detect shelf:{} id:{}".format(self.__is_shelf_detect, self.__is_package_detect))

    def set_pose_callback(self, msg):
        """
        Handle msg form set_pose topic
        Params:
        ==========
        msg: Pose
        """
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

        rospy.loginfo(
            "Current position {0} current_target {1}".format(
                self.group.get_current_pose(), [self.target]
            )
        )
        self.log("Position received")

        time.sleep(1)
        self.handle_command(2)

    def goal_result_callback(self, result_msg):
        """
        Check goal status execute load/unload package
        Params:
        =============
        result_msg: Int32
        """
        try:
            rospy.loginfo("Goal result ")
            self.log("Goal result")
            if result_msg.data:
                if "unload" in self.action_to_do:
                    self.group.set_named_target("ready_1")
                    self.handle_command(1)
                    time.sleep(3)
                    for i in range(3):
                        trans = self.lookup_transform(
                            "fiducial_" + str(self.shelf_id), "dummy"
                        )[0]
                        if trans:
                            if trans.x < 0.329:
                                break
                            rospy.loginfo(trans.x)
                            self.speed_pub.publish(Float32(0.14))
                            delay = (trans.x - 0.329) / 0.14
                            time.sleep(delay + 0.1)
                        else:
                            if self.__is_back_obstacle:
                                time.sleep(3)
                                continue
                            self.speed_pub.publish(Float32(-0.14))
                            time.sleep(0.4)
                        self.speed_pub.publish(Float32(0.0))
                        time.sleep(2)
                    else:
                        if self.count < 3:
                            self.goal_shelf.header.stamp = rospy.Time()
                            self.goal_pub.publish(self.goal_shelf)
                            msg = "Publish goal_shelf finished"
                            self.log(msg)
                            self.count += 1
                        return
                    self.count = 0
                    self.pick_n_place_onboard("fiducial_" + str(self.chosen_id))
                else:
                    self.detect_id = ""
                    if self.move_to_shelf():
                        self.group.set_named_target("ready")
                        self.handle_command(1)
                        time.sleep(3)
                        for i in range(3):
                            if self.detect_id:
                                self.pick_n_place_shelf(self.detect_id)
                                break
                            else:
                                time.sleep(1)
                    else:
                        return
        except:
            self.log("goal_result_callback@Exception", traceback.format_exc())

    def result_callback(self, result_msg):
        """
        Handle msg form result topic, this is for testing
        Params:
        ============
        result_msg: Int32
        """
        try:
            self.log("Result_callback")
            # self.handle_keyboard()
            self.detect_id = ""
            self.chosen_id = input("Chosen id: ")
            if self.move_to_shelf():
                time.sleep(3)
                for i in range(3):
                    self.group.set_named_target("ready")
                    self.handle_command(1)
                    if self.detect_id:
                        self.log("self detect_id: {}".format(self.detect_id))
                        self.pick_n_place_shelf(self.detect_id)
                        break
                    else:
                        time.sleep(1)
            # self.execute_joint(0, 1)
            self.pick_n_place_onboard("fiducial_" + str(self.chosen_id))
            self.group.set_named_target("home")
            self.handle_command(1)
        except:
            self.log("result_callback@Exception", traceback.format_exc())

    def back_obstacle_callback(self, msg):
        """
        Save back_obstacle msg publised from sonar
        Params:
        ============
        msg: Bool
        """
        self.__is_back_obstacle = msg.data

    def yaw_angle_callback(self, msg):
        """
        Get yaw angle now from yaw_angle topic calculate by nav_excute
        Params:
        ==========
        msg: Float32
        """
        self.yaw_angle_deg = np.rad2deg(msg.data)

    def lookup_transform(self, target_frame, source_frame, look_for_rot=False):
        """
        Looking up for transform form source_frame to target frame in tf
        Params:
        ==================
        target_frame, source_frame: tf frame"""
        yaw_tmp = []
        for i in range(5):
            try:
                trans = self.tfBuffer.lookup_transform(
                    source_frame, target_frame, rospy.Time()
                )  # lookup_transform(frame dich, frame goc, timeout)
                translation = trans.transform.translation
                rotation = trans.transform.rotation
                rotation = quat2euler([rotation.w, rotation.x, rotation.y, rotation.z])
                yaw_tmp += [rotation[2]]
                if not look_for_rot:
                    break
                if i == 4:
                    rotation[2] = np.median(yaw_tmp)
                    break
            except:
                self.log("No transform")
        else:
            return [], []
        return translation, rotation

    def set_robot_perpendicular(self, id):
        """
        Rotate robot perpendicular to shelf or package face
        Params:
        ============
        id: shelf_id or package id (Int)"""
        rot = []
        rot = self.lookup_transform("fiducial_" + str(id), "dummy", look_for_rot=True)[1]
        if not rot:
            return False
        rot_yaw = np.rad2deg(rot[2])
        self.log("rot_yaw: {}".format(rot_yaw))
        if (
            not -114 > rot_yaw > -105
        ):  ## the range where the package face is considered to be perpendicular to robot
            self.set_rotate_angle_pub.publish(109 + rot_yaw)
            time.sleep(6)
            self.log("Aligning {}".format(109 + rot_yaw))
            return False
        else:
            self.log("Already perpendiculation")
            return True

    def decrease_y_distance(self, delta_y):
        """
        Align robot straight with the package
        Params:
        ==============
        delta_y: distace from robot to target package id in x axis (Float)"""
        try:
            start_yaw = self.yaw_angle_deg
            angle = Int32(90) if delta_y > 0 else Int32(-90)
            self.set_rotate_angle_pub.publish(angle)
            time.sleep(10)
            rospy.loginfo("Spin 90")
            self.set_move_distance_pub.publish(Float32(abs(delta_y)))
            time.sleep(5)
            rospy.loginfo("Go straight 90")
            self.set_rotate_angle_pub.publish(start_yaw - self.yaw_angle_deg)
            self.log(
                "angle: {}; return_angle: {}".format(
                    angle, start_yaw - self.yaw_angle_deg
                )
            )
            time.sleep(10)
        except:
            self.log("decrease_y_distance@Exception", traceback.format_exc())

    def spin_joint_0_finding(self, id):
        """
        If no transfrom rotate arm left and right to look for the target id
        Params:
        ==============
        id: target id (Int)"""
        for i in range(7):
            trans, rot = [], []
            trans, rot = self.lookup_transform("fiducial_" + str(id), "dummy")
            if trans:
                self.log("spin_joint_0_finding finish success")
                return True
            joint_0_angle = self.group.get_current_joint_values()[0]
            ## rotate left and right, each time in the loop move further
            joint_0_angle += -0.2 * (i + 1) * (-1) ** i
            self.group.set_joint_value_target(
                "joint_0", 0.7 if joint_0_angle >= 0.7 else joint_0_angle
            )
            self.handle_command(1)
            time.sleep(3)
        else:
            self.log("Fail_fiding_id")
            self.group.set_named_target("home")
            self.handle_command(1)
            return False

    def move_to_shelf(self):
        """Moving robot to selected shelf code"""
        try:
            self.group.set_named_target("ready_1")
            self.handle_command(1)
            time.sleep(3)
            set_perpendicular = False
            ### move closer to the shelf; if lost track of the package, move out and try again
            for i in range(8):
                trans_ = []
                rospy.loginfo("i: {}".format(i))
                pre_trans = trans_
                trans_ = self.lookup_transform("fiducial_" + str(self.shelf_id), "link_1")[0]
                rospy.loginfo("trans_ look: {}".format(trans_))
                if not trans_:
                    set_perpendicular = False
                    for i in range(2):
                        rospy.loginfo("1")
                        if not self.spin_joint_0_finding(self.shelf_id):
                            self.log("Move to shelf return, no transform")
                            return
                        if not set_perpendicular:
                            self.set_robot_perpendicular(self.shelf_id)
                            set_perpendicular = True
                    self.log("Retrying finding transformation")
                    continue

                coeff = [-13, 41, -11]
                trans_.y += np.dot(coeff, [[trans_.x ** 2], [trans_.x], [1]])[0] * 0.01
                # trans_.y += 0.03
                if not set_perpendicular:
                    self.set_robot_perpendicular(self.shelf_id)
                    set_perpendicular = True
                    i -= 1
                    continue

                if trans_ == pre_trans: 
                    i -= 1
                    continue
                rospy.loginfo("trans x: {}, trans y : {}".format(trans_.x, trans_.y))
                if abs(trans_.y) > 0.06:
                    rospy.loginfo("4")
                    self.decrease_y_distance(trans_.y)
                    self.group.set_named_target("ready_1")
                    self.handle_command(1)
                    set_perpendicular = False
                    time.sleep(3)
                    continue
                # trans_.x += -0.5 * trans_.y
                rospy.loginfo("trans_x look: {}".format(trans_.x))

                self.log("Transform_shelf_2: {}".format(trans_))
                if 0.24 <= trans_.x <= 0.29:
                    break
                # elif trans_.x > 0.38:
                #     self.set_move_distance_pub.publish(Float32(0.18))
                #     set_perpendicular = False
                #     i -= 1
                # else:
                self.set_move_distance_pub.publish(Float32(trans_.x - 0.25))
                break
            else:
                rospy.loginfo("5")
                self.log("Cannot get to the shelf")
                self.group.set_named_target("home")
                self.handle_command(1)
                ## moving out of the shelf area
                self.set_move_distance_pub.publish(Float32(-0.15))
                return False
            rospy.loginfo("5")
            self.log("Finish moving close to the shelf")
            return True
        except:
            self.log("move_to_shelf@Excetption", traceback.format_exc())

    def pick_n_place_shelf(self, target_frame):
        """
        Pick the package from shelf to the robot
        Params:
        ==========
        target_frame: the package target frame id (tf_frame)"""
        try:
            self.log("Start pick and place shelf")
            ## stop the robot in case resducial action before
            self.speed_pub.publish(Float32(0.0))
            ## execute pick package
            shift_to_package = 0.057
            for i in range(3):
                self.group.set_named_target("ready")
                self.handle_command(1)
                time.sleep(2)
                trans = []
                if self.spin_joint_0_finding(int(target_frame.split("_")[1])):
                    trans = self.lookup_transform(target_frame, "dummy")[0]
                else:
                    self.log("Cannot find package")
                    return
                ## release gripper
                self.calc_gripper_angle(release=True)
                time.sleep(3)
                trans.x += -0.5 * trans.y - 0.10
                trans.y *= 1.1

                rospy.loginfo("position = {}".format([trans.x, trans.y, trans.z]))
                yaw = (
                    np.arctan((trans.y - 0.022) / (trans.x - 0.095)) - 1.495
                )  # 1.495 is the offset angle between start and 90deg position of servo
                pose = [trans.x, trans.y, trans.z] + [0.0, 0.0, yaw]

                self.log("Target pose: {}".format(pose))

                self.group.set_pose_target(pose)
                a = self.group.plan()
                if not a.joint_trajectory.points:
                    rospy.loginfo("no plan")
                    self.log("no plan 1")
                    continue

                self.handle_command(1)
                time.sleep(1)
                rospy.loginfo("pose_1: {}".format(self.group.get_current_pose()))
                ## move the arm straight to the package
                ## after testing i picked 5.4cm cuz the distance from package and camera isn't exactly
                for i in range(3):
                    self.group.shift_pose_target(0, shift_to_package)
                    a = self.group.plan()
                    if not a.joint_trajectory.points:
                        shift_to_package -= 0.003
                        rospy.loginfo("no plan")
                        self.log("no plan 2")
                        continue
                    rospy.loginfo("pose_shift: {}".format(self.group.get_current_pose()))
                    break
                else:
                    continue

                rospy.loginfo("pose_2: {}".format(self.group.get_current_pose()))
                self.log("pose_2: {}".format(self.group.get_current_pose()))
                self.handle_command(1)
                self.execute_joint(1, 0.1)
                self.execute_joint(2, 0.05)
                time.sleep(0.5)
                self.calc_gripper_angle(self.package_width, False)
                time.sleep(3)

                ## move to ready position to check if package is picked or not
                self.group.shift_pose_target(2, 0.04)
                rospy.loginfo("pose_3: {}".format(self.group.get_current_pose()))
                self.handle_command(1)
                self.group.set_named_target("ready")
                self.handle_command(1)
                time.sleep(2)
                trans = self.lookup_transform(target_frame, "dummy")[0]
                if not trans:
                    break
            else:
                self.log("Cannot pick package on shelf")
                ## moving out of the shelf area
                self.set_move_distance_pub.publish(Float32(-0.2))
                return

            ## move arm higher and get to the deck
            self.execute_joint(3, 0.3)
            self.execute_joint(2, -0.3)
            if not self.package_count:
                self.group.set_named_target("load_2")
            else:
                self.group.set_named_target("load_1")
            self.handle_command(1)

            ## bcuz the load positions are set higher than the deck to avoid collision
            #  so i need to shift it to lower position
            self.handle_command(1)
            self.execute_joint(2, 0.2)
            self.calc_gripper_angle()
            time.sleep(2)

            ## move out and go back to home position and wait for another command
            # self.group.shift_pose_target(2, 0.06)
            # self.handle_command(1)
            self.execute_joint(2, -0.25)
            self.group.set_named_target("home")
            self.handle_command(1)
            if self.package_count < 2:
                self.package_count += 1  # count the package on the deck

            ## moving out of the shelf area
            # self.set_move_distance_pub.publish(Float32(-0.2))
        except:
            self.log("pick_n_place_shelf@Exception", traceback.format_exc())

    def pick_n_place_onboard(self, target_frame, target_shelf="fiducial_1"):
        """
        Pick and place package from robot to the selected shelf
        Params:
        ===========
        target_frame: (tf_frame) frame id of the target package"""
        try:
            self.group.set_named_target("find_onboard")
            self.handle_command(1)
            time.sleep(3)
            for i in range(3):
                if self.spin_joint_0_finding(int(target_frame.split("_")[1])):
                    ## moving out of the shelf area
                    trans = self.lookup_transform(target_frame, "dummy")[0]
                else:
                    self.log("Cannot find package")
                    return
                self.calc_gripper_angle()
                time.sleep(3)
                rospy.loginfo("position = {}".format([trans.x, trans.y]))
                ## the transx, transy error is linear belong to each other after testing
                trans.x += -0.5 * trans.y + 0.04
                if trans.y < 0.02:
                    trans.y += 0.025 * trans.x / 0.114
                elif trans.y >0.046:
                    trans.y *= 0.8

                yaw = (
                    np.arctan((trans.y - 0.022) / (trans.x - 0.095)) + 1.648
                )  ## 1.648 is the 90 angle position of ther arm
                pose = [trans.x, trans.y, 0.2] + [0.0, 0.0, yaw]
                rospy.loginfo("Target pose: {}".format(pose))

                self.log("Target pose: {}".format([pose[0], pose[1], pose[5]]))

                ## move arm to target pose
                self.group.set_pose_target(pose)
                a = self.group.plan()
                time.sleep(0.5)
                if not a.joint_trajectory.points:
                    rospy.loginfo("plannig")
                    pose[5] += 0.025
                    self.group.set_pose_target(pose)
                    a = self.group.plan()
                    if abs(pose[5] - yaw) > 0.2:
                        break
                self.handle_command(1)

                
                rospy.loginfo("Current joint: {}".format(self.group.get_current_joint_values()))
                joint_angle = self.group.get_current_joint_values()
                joint_angle[3] -= 0.06
                joint_angle[2] += 0.21
                joint_angle[1] += 0.01
                self.group.set_joint_value_target(joint_angle)
                self.handle_command(1)

                ## pickup the package and check if package is picked or not
                # self.execute_joint(2, 0.03)
                self.calc_gripper_angle(self.package_width, release=False)
                time.sleep(3)
                self.group.shift_pose_target(2, 0.07)
                self.handle_command(1)
                rospy.loginfo("Current joint: {}".format(self.group.get_current_joint_values()))
                self.execute_joint(0, 0.4)
                time.sleep(1)
                rospy.loginfo("Current position: {}".format(self.group.get_current_pose()))
                self.group.set_named_target("find_onboard")
                self.handle_command(1)
                time.sleep(3)
                trans, rot = self.lookup_transform(target_frame, "dummy")
                if not trans:
                    break
            else:
                self.log("Cannot pick package onboard")
                ## moving out of the shelf area
                # self.set_move_distance_pub.publish(Float32(-0.2))
                return

            ## move joint_1 back
            self.execute_joint(1, 1)
            if self.package_count > 0:
                self.package_count -= 1

            ## put package on shelf and return home
            self.group.set_named_target("ready")
            self.handle_command(1)
            self.execute_joint(3, -0.25)
            self.execute_joint(2, -0.175)
            self.group.set_named_target("put_on_shelf")
            self.handle_command(1)
            self.group.shift_pose_target(2, -0.05)
            self.handle_command(1)
            self.execute_joint(3, 0.15)
            self.calc_gripper_angle()
            time.sleep(3)
            self.group.shift_pose_target(2, 0.05)
            self.handle_command(1)
            self.group.set_named_target("home")
            self.handle_command(1)

            ## moving out of the shelf area
            self.set_move_distance_pub.publish(Float32(-0.2))
        except:
            self.log("pick_n_place_onboard@Exception", traceback.format_exc())

    def handle_keyboard(self):
        """
        Get and process the input selection from user
        """
        for attemp in range(10):
            try:
                # package = input("Choose your package id: ")
                self.action_to_do = input("Choose your action (load/unload): ")
                if not "load" in self.action_to_do:
                    continue
                if "unload" in self.action_to_do:
                    try:
                        self.chosen_id = int(input("Choose your package to unload: "))
                    except:
                        print("Please input valid integer value")
                        continue
                shelf = input("Choose shelf id: ")
                self.shelf_id = int(shelf[1]) if int(shelf[1]) in [1, 2, 3] else 0
                self.goal_shelf.pose.position = shelf_position(shelf)
                self.goal_shelf.pose.orientation = shelf_orientation(shelf)
                self.goal_shelf.header.stamp = rospy.Time()
                self.goal_pub.publish(self.goal_shelf)
                msg = "Publish goal_shelf finished"
                self.log(msg)
                rospy.loginfo(msg)
                break
            except:
                rospy.logwarn(traceback.format_exc())
        else:
            return False

    def handle_command(self, command):
        """
        Choose a command for moveit to execute
        Params:
        ===============
        command: (Int)  0: stop arm
                        1: execute to target selected before
                        2: execute to target saved in self.targets
        """
        try:
            self.log("command: %f", command)
            if command == 0:
                self.group.stop()
                self.group.clear_pose_targets()
                time.sleep(0.5)
            elif command == 1:
                self.log("Executing to joint_target")
                self.group.set_goal_tolerance(0.005)
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
            self.log("handle_command@Exception", traceback.format_exc())

    def calc_gripper_angle(self, package_width=50, release=True):
        """
        This is specified for my robot only
        =====> calculate servo angle suitable for each package width
        my equation: y = -1.09 * x + 60.3
                     y: gripper angle (deg)
                     x: width of the package (mm)
        Params:
        ==============
        package_width:(Int) mm width of the package
        release: (Bool)     True: release the gripper
                            False: pick the package
        """

        if release:
            self.gripper_state.publish(Int32(6))
        else:
            joint_angle = -1.09 * package_width + 63.3
            if joint_angle <= np.rad2deg(0.15):
                joint_angle = np.rad2deg(0.15)
            elif joint_angle >= np.rad2deg(0.9):
                joint_angle = np.rad2deg(0.9)
            self.gripper_state.publish(Int32(joint_angle + 8))

    def execute_joint(self, joint, angle, set_joint=False):
        """
        Add/minus or set specific joint angle with a specific value
        Params:
        =============
        joint: (Int) joint want to execute
        angle: (Int) angle to rotate
        set_joint: (Bool)   True: angle is the position want to rotate to rotate
                            False: angle is the angle want to add to current position
        """
        if not angle:
            return
        joint_angle = self.group.get_current_joint_values()
        joint_angle[joint] = joint_angle[joint] + angle if not set_joint else angle
        self.group.set_joint_value_target(joint_angle)
        self.handle_command(1)

    def log(self, *arg):
        """
        logging to file.
        """
        # log(self.root_node, arg)
        now = int(time.time() * 1000)  # milliseconds
        msg = []
        msg.append(str(now))
        for x in arg:
            msg.append("[" + str(x) + "]")
        msg.append("\n")
        # self.root_node.get_logger().info("".join(msg))
        self.__file_log.write(" ".join(msg))


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("Arm_moveit", anonymous=True)
    moveit = moveit_handle()
    rospy.spin()
    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
