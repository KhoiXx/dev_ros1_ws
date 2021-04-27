#!/usr/bin/env python

import rospy
import sys
import time

from Robot import Robot
rosserial_port = '/dev/ttyTHS1'

class Thesis_robot:
    def __init__(self, port , baudrate):
        self.thesis_robot = Robot(port, baudrate)
        time.sleep(1)

def main():
    try:
        rospy.init_node("Thesis_robot")
        rospy.loginfo("ROS Node Thesis_robot")
        try:
            Thesis_robot(port = rosserial_port, baudrate = 115200)
        except Exception as ex:
            sys.exit(str(ex))
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Thesis_robot")
        rospy.on_shutdown()
