#!/usr/bin/env python

import rospy
import sys
import time

from Robot import Robot
from nav_execute import Navigation

PORT = '/dev/ttyTHS1'

class Thesis_robot(object):
    def __init__(self, port , baudrate):
        self.thesis_robot = Robot(port, baudrate)
        time.sleep(1)
        self.navigation = Navigation(self.thesis_robot)

def main(port, baud):
    try:
        rospy.init_node("Thesis_robot")
        rospy.loginfo("ROS Node Thesis_robot")
        try:
            Thesis_robot(port = port, baudrate = baud)
            rospy.loginfo("Thesis_robot is connected to port THS1")
        except Exception as ex:
            sys.exit(str(ex))
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Thesis_robot")
        rospy.on_shutdown()

if __name__=='__main__':
    rosserial_port = rospy.get_param('~port', '/dev/ttyTHS1')
    baud = int(rospy.get_param('~baud','115200'))
    main(rosserial_port, baud)
