#!/usr/bin/env python

import os
import sys
import rospy


class Navigation():
    def __init__(self,port = rosserial_port,baudrate = 115200):
        self.robot = 