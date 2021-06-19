#!/usr/bin/env python

import traceback
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, PoseStamped, PoseWithCovarianceStamped
import time

from pynput.keyboard import Key, Listener
from key_mapping import Key_mapping
import datetime

ROS_WS = '/home/khoixx/dev_ros1_ws'
LOG_FILE_PATH = ROS_WS + '/log/arm_log/'

class moveit_handle:
    def __init__(self):
        self.key_pressed = None
        log_info = [
            LOG_FILE_PATH,
            datetime.datetime.now().strftime('arm_real %d%m_%H:%M'),
            '.log'
        ]
        self.__file_log = open("".join(log_info), "w")

        self.robot = moveit_commander.RobotCommander()
        self.scence = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander('robotarm')
        self.group.set_named_target("home")
        self.group.go()
        self.group.stop()
        time.sleep(3)
        self.group.clear_pose_targets()
        self.handle_command(2)
        self.target = Pose()
        rospy.Subscriber('target_position', Pose, self.callback)
        
        rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)

        self.log('Finish initializing arm_moveit')
        rospy.loginfo('Finish initializing arm_moveit')

    def callback(self,msg):
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
        with Listener(on_press = self.press_handle) as listener:
            listener.join()
    
    def press_handle(self, key):
        if key == Key.space:
            self.key_pressed = 0
        elif key == Key_mapping.PLAN_ARM:
            self.key_pressed = 1
        elif key == Key_mapping.EXECUTE_ARM:
            self.key_pressed = 2
        else:
            self.key_pressed = None

        if key == Key.shift:
            pose_current = self.group.get_current_pose()
            print(pose_current)
        rospy.loginfo('key pressed: %s',key)
        self.log('key: %s',key)
        if key == Key.esc:
            return False

    def handle_command(self, command):
        
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

        
