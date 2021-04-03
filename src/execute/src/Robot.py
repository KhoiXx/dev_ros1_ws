#!/usr/bin/env python

#import section
import serial as ser
import os
import numpy as numpy
import sys
import rospy

from key_mapping import Key_mapping
from serial import Serial

#variables
rosserial_port = '/dev/ttyTHS1'

#main code

class Robot:
    def __init__(self,port = rosserial_port,baudrate = 115200):
        
