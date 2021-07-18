#!/bin/bash

gnome-terminal --tab -- bash -c "cd /home/khoixx/dev_ros1_ws; roslaunch robot_bringup robot_imu.launch; exec bash"
gnome-terminal --tab -- bash -c "sleep 5; roslaunch robot_bringup robot_base.launch; exec bash"
gnome-terminal --tab -- bash -c "sleep 6; roslaunch robot_bringup arm_control.launch; exec bash"
gnome-terminal --tab -- bash -c  "sleep 14; roslaunch robot_bringup robot_rtabmap.launch; exec bash"
gnome-terminal --tab -- bash -c "sleep 60; roslaunch robot_bringup robot_with_camera.launch; exec bash"
gnome-terminal --tab -- bash -c "sleep 80; roslaunch robot_bringup pi_cam.launch; exec bash" 
