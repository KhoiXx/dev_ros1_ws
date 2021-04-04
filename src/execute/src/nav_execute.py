#!/usr/bin/env python

import os
import sys
import rospy

from Robot import Robot

class Navigation:
    def __init__(self, _robot: Robot):
        self.__robot = _robot
        