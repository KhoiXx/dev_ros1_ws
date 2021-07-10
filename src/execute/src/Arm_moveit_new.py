#!/usr/bin/env python

import traceback
import rospy
import sys
import copy
import moveit_commander
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

ROS_WS = "/home/khoixx/dev_ros1_ws"
LOG_FILE_PATH = ROS_WS + "/log/arm_log/"
GOAL_STATUS_SUCCESS = 3
GOAL_STATUS_ABORT = 4


class Fiducials_id:
    PACKAGE_1 = [11, 21, 31, 41, 51, 61, 71, 81, 91]
    PACKAGE_2 = [12, 22, 32, 42, 52, 62, 72, 82, 92]
    PACKAGE_3 = [13, 23, 33, 43, 53, 63, 73, 83, 93]
    shelf_pose = [1, 2, 3]


PACKAGE_WIDTH_MIN = 10
PACKAGE_WIDTH_MAX = 45


def position(id="A1"):
    case = {
        "A1": Point(2.212, -0.467, 0.00),
        "A2": Point(2.22, -0.643, 0.00),
        "A3": Point(2.244, -0.771, 0.00),
        "B1": Point(3.34, 1.25, 0.00),
        "B2": Point(3.43, 1.25, 0.00),
        "B3": Point(3.52, 1.25, 0.00),
    }
    return case.get(id, Point(2.292, -0.467, 0.00))


def orientation(id="A1"):
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
        self.key_pressed = None
        log_info = [
            LOG_FILE_PATH,
            datetime.datetime.now().strftime("arm_real %d%m_%H:%M"),
            ".log",
        ]
        self.__file_log = open("".join(log_info), "w")
        self.__init_variables()
        self.__init_topic()
        self.__init_arm()
        self.__robot_base = Robot()
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        self.log("Finish initializing arm_moveit")
        rospy.loginfo(
            "Finish initializing arm_moveit {}".format(
                self.group.get_end_effector_link()
            )
        )

    def __init_topic(self):
        # rospy.Subscriber("target_position", Pose, self.set_pose_callback)
        rospy.Subscriber("/mytopic/back_obstacle", Bool, self.back_obstacle_callback)
        rospy.Subscriber("/mytopic/nav_success", Bool, self.goal_result_callback)
        rospy.Subscriber("/mytopic/result", Int32, self.result_callback)
        rospy.Subscriber(
            "/fiducial_transforms", FiducialTransformArray, self.aruco_transform
        )
        rospy.Subscriber("/mytopic/yaw_angle", Float32, self.yaw_angle_callback)

        # find_pose_pub = rospy.Publisher("/finding_pose", Float32, queue_size=10)
        self.gripper_state = rospy.Publisher("/gripper_state", Int32, queue_size=10)
        self.set_rotate_angle_pub = rospy.Publisher(
            "/mytopic/set_rotate_angle", Int32, queue_size=10
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
        self.__is_package_onboard = False
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
        for i in msg.transforms:
            if i.fiducial_id == self.shelf_id:
                # self.log("detect shelf")
                self.__shelf_id_time = rospy.get_rostime()
            if i.fiducial_id == self.chosen_id:
                self.__chosen_id_time = rospy.get_rostime()
                self.package_id = i.fiducial_id
                # self.log("detect_id")
            self.detect_id = "fiducial_" + str(i.fiducial_id)
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

    # def find_shelf(self, target_id):
    #     self.group.set_named_target("ready_1")
    #     self.handle_command(1)
    #     time.sleep(3)
    #     for i in range(5):
    #         trans = self.lookup_transform("fiducial_" + str(self.shelf_id), "dummy")[0]
    #         if trans is not None:


    def goal_result_callback(self, result_msg):
        """
        Check goal status and correct robot heading
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
                        trans = self.lookup_transform("fiducial_" + str(self.shelf_id), "dummy")[0]
                        if trans is not None:
                            if trans.x < 0.329:
                                break
                            rospy.loginfo(trans.x)
                            # self.__robot_base.set_speed([0.14, 0.14])
                            self.speed_pub.publish(Float32(0.14))
                            delay = (trans.x - 0.329) / 0.14
                            time.sleep(delay+0.1)
                        else:
                            if self.__is_back_obstacle:
                                time.sleep(3)
                                continue
                            # self.__robot_base.set_speed([-0.14, -0.14])
                            self.speed_pub.publish(Float32(-0.14))
                            time.sleep(0.4)
                        # self.__robot_base.set_stop()
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
                    self.group.set_named_target("ready")
                    self.handle_command(1)
                    time.sleep(3)

                    ## move backward to have wider angle
                    for i in range(3):
                        if self.detect_id:
                            break
                        if self.__is_back_obstacle:
                            time.sleep(3)
                            continue
                        # self.__robot_base.set_speed([-0.14, -0.14])
                        self.speed_pub.publish(Float32(-0.14))
                        time.sleep(0.5)
                        self.__robot_base.set_stop()
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
                    self.pick_n_place_shelf(self.detect_id)
        except:
            self.log("goal_result_callback@Exception", traceback.format_exc())

    def result_callback(self, result_msg):
        """
        Check goal status and correct robot heading
        """
        try:
            self.log("Result_callback")
            self.move_to_shelf()
        except:
            self.log("result_callback@Exception", traceback.format_exc())

    def back_obstacle_callback(self, msg):
        """save back_obstacle msg publised from sonar"""
        self.__is_back_obstacle = msg.data

    def yaw_angle_callback(self, msg):
        self.yaw_angle_deg = np.rad2deg(msg.data) 

    def lookup_transform(self, target_frame, source_frame):
        for i in range(5):
            try:
                trans = self.tfBuffer.lookup_transform(
                    source_frame, target_frame, rospy.Time()
                )  # lookup_transform(frame dich, frame goc, timeout)
                translation = trans.transform.translation
                rotation = trans.transform.rotation
                rotation = quat2euler([rotation.w, rotation.x, rotation.y, rotation.z])
                break
            except:
                self.log("No transform")
        else:
            return None, None
        return translation, rotation

    def move_to_shelf(self):
        "Moving robot to selected shelf code"
        self.group.set_named_target("ready_1")
        self.handle_command(1)
        time.sleep(3)
        for i in range(8):
            trans = self.lookup_transform("fiducial_" + str(self.shelf_id), "dummy")[0]
            self.log("Transform_shelf_1: {}".format(trans))
            if trans is not None:
                trans.y += 0.08
                break
            joint_0_angle = self.group.get_current_joint_values()[0]
            joint_0_angle += 0.2 * (i+1) * (-1) ** i
            self.group.set_joint_value_target("joint_0", joint_0_angle)
            self.handle_command(1)
            time.sleep(2)
        else:
            self.group.set_named_target("home")
            self.handle_command(1)
            return False
        start_yaw = self.yaw_angle_deg
        if abs(trans.y) > 0.05:
            angle = Int32(80) if trans.y > 0 else Int32(-80) 
            self.set_rotate_angle_pub.publish(angle)
            time.sleep(6)
            self.speed_pub.publish(Float32(0.14))
            time_delay = (trans.y / 0.14)
            time.sleep(time_delay)
            self.speed_pub.publish(Float32(0.0))
            self.set_rotate_angle_pub.publish(start_yaw - self.yaw_angle_deg)
            time.sleep(3)
        count = 0
        for i in range(7):
            trans, rot = self.lookup_transform("fiducial_" + str(self.shelf_id), "dummy")
            if trans is None:
                if not self.__is_back_obstacle:
                    self.speed_pub.publish(Float32(-0.14))
                count +=1
                time.sleep(1)
                self.speed_pub.publish(Float32(0.0))
                continue
            self.log("Transform_shelf_2: {}, {}".format(trans, rot))
            rot[2] = np.rad2deg(rot[2])
            if not -114 > rot[2] > -105:
                self.set_rotate_angle_pub.publish(107 - rot[2])
                time.sleep(3)
            trans.x += 0.5 * trans.y
            if 0.3 <= trans.x <= 0.38:
                self.log("In the working range")
                break
            elif trans.x > 0.38:
                time_delay = abs(trans.x - 0.38) / 0.14
                speed = 0.14
            elif trans.x < 0.3:
                time_delay = abs(trans.x - 0.3) / 0.14
                speed = -0.14
                if self.__is_back_obstacle: continue
            self.speed_pub.publish(Float32(speed))
            time.sleep(time_delay)
            self.speed_pub.publish(Float32(0.0))
        else:
            self.log("Cannot get to the shelf")
            self.group.set_named_target("home")
            self.handle_command(1)
            return False
        self.log("Finish moving close to the shelf")
        return True
            
    def pick_n_place_shelf(self, target_frame):
        # self.__robot_base.set_stop()
        self.speed_pub.publish(Float32(0.0))
        self.log("Start pick and place shelf")

        ## check if robot is too close or too far from package
        for i in range(5):
            trans = self.lookup_transform(target_frame, "dummy")[0]
            # self.log("Transform: x:{} y:{} z:{}".format(trans.x, trans.y, trans.z))
            # rospy.loginfo("Transform: x:{} y:{} z:{}".format(trans.x, trans.y, trans.z))
            if trans is None:
                # self.__robot_base.set_speed([-0.14, -0.14])
                self.speed_pub.publish(Float32(-0.14))
                time.sleep(0.4)
            elif trans.x > 0.38:
                # self.__robot_base.set_speed([0.14, 0.14])
                self.speed_pub.publish(Float32(0.14))
                delay = (trans.x - 0.388) / 0.14
                time.sleep(delay)
            else:
                break
            # self.__robot_base.set_stop()
            self.speed_pub.publish(Float32(0.0))
            time.sleep(0.5)
        else:
            self.log("No transformation")
            rospy.loginfo("No transformation")
            return False

        ## execute pick package
        for i in range(3):
            self.calc_gripper_angle()
            time.sleep(3)
            trans.x -= 0.12  # set the goal 12cm away from the package

            rospy.loginfo("position = {}".format([trans.x, trans.y, trans.z]))
            yaw = (
                np.arctan(trans.y / trans.x) - 1.49
            )  # 1.49 is the offset angle between start and 90deg position of servo
            pose = [trans.x, trans.y, trans.z] + [0.0, 0.0, yaw]

            self.log("Target pose: {}".format(pose))

            self.group.set_pose_target(pose)
            a = self.group.plan()

            ## if moveit cannot find the valid trajectory try to increase the yaw angle
            ## cuz the yaw caculate above isn't perfectly correct
            while not a.joint_trajectory.points:
                rospy.loginfo("plannig")
                pose[5] += 0.025
                self.group.set_pose_target(pose)
                a = self.group.plan()
                if abs(pose[5] - yaw) > 0.2:
                    break
            self.handle_command(1)

            ## move the arm straight to the package
            ## after testing i picked 4.9cm cuz the distance from package and camera isn't exactly
            self.group.shift_pose_target(0, 0.054)
            self.handle_command(1)
            self.calc_gripper_angle(self.package_width, False)
            time.sleep(3)

            ## move to ready position to check if package is picked or not
            self.group.shift_pose_target(2, 0.02)
            self.handle_command(1)
            self.group.set_named_target("ready")
            self.handle_command(1)
            trans = self.lookup_transform(target_frame, "dummy")[0]
            time.sleep(2)
            if trans == None:
                break
        else:
            return

        self.execute_joint(2, -0.1)
        if not self.package_count:
            self.group.set_named_target("load_2")
        else:
            self.group.set_named_target("load_1")
        self.handle_command(1)

        ## bcuz the load positions are set higher than the deck to avoid collision
        #  so i need to shift it to lower position
        self.group.shift_pose_target(2, -0.02)
        self.handle_command(1)
        self.execute_joint(2, 0.05)
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

    def pick_n_place_onboard(self, target_frame, target_shelf="fiducial_1"):

        self.group.set_named_target("find_onboard")
        self.handle_command(1)
        time.sleep(3)

        for i in range(1, 4):
            trans, rot = self.lookup_transform(target_frame, "dummy")
            if trans == None:
                joint_angle = self.group.get_current_joint_values()
                joint_angle[0] += (0.2 * i) * (-1) ** i
                self.group.set_joint_value_target(joint_angle)
                self.handle_command(1)
                time.sleep(3)
            else:
                break
        else:
            return
        for i in range(3):
            self.calc_gripper_angle()
            time.sleep(3)
            rospy.loginfo("position = {}".format([trans.x, trans.y, rot[2]]))
            trans.x += -0.5 * trans.y + 0.06
            if trans.y < 0.03:
                trans.y += 0.025 * trans.x / 0.114

            yaw = np.arctan((trans.y - 0.022) / (trans.x - 0.095)) + 1.648
            pose = [trans.x, trans.y, 0.2] + [0.0, 0.0, yaw]
            rospy.loginfo("Target pose: {}".format(pose))

            self.log("Target pose: {}".format([pose[0], pose[1], pose[5]]))

            self.group.set_pose_target(pose)
            a = self.group.plan()
            time.sleep(0.5)
            while not a.joint_trajectory.points:
                rospy.loginfo("plannig")
                pose[5] += 0.025
                self.group.set_pose_target(pose)
                a = self.group.plan()
                if abs(pose[5] - yaw) > 0.2:
                    break
            self.handle_command(1)

            joint_angle = self.group.get_current_joint_values()
            joint_angle[3] -= 0.08
            joint_angle[2] += 0.19
            self.group.set_joint_value_target(joint_angle)
            self.handle_command(1)
            # self.group.shift_pose_target(0, -0.03)
            # self.handle_command(1)
            self.calc_gripper_angle(self.package_width, False)
            time.sleep(3)
            self.execute_joint(2, 0.36)
            self.group.set_named_target("find_onboard")
            self.handle_command(1)
            time.sleep(2)
            trans, rot = self.lookup_transform(target_frame, "dummy")
            if trans == None:
                break
        else:
            return
        self.execute_joint(1, 1)
        if self.package_count > 0:
            self.package_count -= 1
        self.group.set_named_target("ready")
        self.handle_command(1)
        self.execute_joint(3, -0.22)
        self.group.set_named_target("put_on_shelf")
        self.handle_command(1)
        self.execute_joint(3, 0.15)
        self.calc_gripper_angle()
        time.sleep(3)
        self.group.shift_pose_target(0, -0.06)
        self.handle_command(1)
        self.group.set_named_target("home")
        self.handle_command(1)

    def handle_keyboard(self):
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
                self.shelf_id = int(shelf[1]) if int(shelf[1]) in [1,2,3] else 0
                self.goal_shelf.pose.position = position(shelf)
                self.goal_shelf.pose.orientation = orientation(shelf)
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
        """Choose a command for moveit to execute"""
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
        """This is specified for my robot only
        =====> calculate servo angle suitable for each package width
        my equation: y = -1.09 * x + 60.3
                     y: gripper angle (deg)
                     x: width of the package (mm)"""

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
        """Add/minus or set specific joint angle with a specific value"""
        if not joint or not angle:
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
    # moveit.handle_keyboard()
    # if not moveit.press_handle():
    rospy.spin()
    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
