#!/usr/bin/env python
# license removed for brevity
import rospy
import serial as ser
import struct,time
from std_msgs.msg import String

sp = ser.Serial('/dev/ttyTHS1',115200,timeout=0.5)

def talker(serial):
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        hello_str = "S B 00240124501 00045178200 E" #% rospy.get_time()
        if(serial.isOpen() == False):
            serial.open()
        serial.write(hello_str)
        time.sleep(1)
	rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker(sp)
    except rospy.ROSInterruptException:
        pass
