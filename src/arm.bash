#!/bin/bash

gnome-terminal --tab -e 'sh -c " roslaunch robot_bringup arm_control.launch; exec bash" --wait'
gnome-terminal --tab -e 'sh -c "sleep 14; roslaunch robot_bringup pi_cam.launch; exec bash" '
# gnome-terminal --tab -e 'sh -c "sleep 14;  rosrun execute Arm_moveit.py; exec bash" '