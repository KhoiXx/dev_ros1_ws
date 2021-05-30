#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from math import pi
import time, threading, traceback


def init_pose(x=0.0, y=0.0, z=0.0, w=1.0):
    try:
        # x=-2.07
        # y=-0.373
        # z=0.0
        # w=1.0
        pose_publish = rospy.Publisher("/rtabmap/initialpose", PoseWithCovarianceStamped, queue_size=20)
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = rospy.Time.now()
        init_pose.pose.pose.position.x = round(float(x), 8)
        init_pose.pose.pose.position.y = round(float(y), 8)
        init_pose.pose.pose.orientation.z = round(float(z), 8)
        init_pose.pose.pose.orientation.w = round(float(w), 8)
        init_pose.pose.covariance[6 * 0 + 0] = 0.25
        init_pose.pose.covariance[6 * 1 + 1] = 0.25
        init_pose.pose.covariance[6 * 5 + 5] = (pi * pi) / (12.0 * 12.0)
        pose_publish.publish(init_pose)
    except Exception as ex:
        traceback.print_exc()

def main():
    try:
        rospy.init_node('init_pose')
        rospy.loginfo("ROS Node Set initialization pose")
        init_pose()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down init_pose node")
        rospy.on_shutdown()

if __name__ == '__main__':
    main()