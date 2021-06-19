#!/bin/bash

gnome-terminal --tab -e 'sh -c " roslaunch arm_moveit demo.launch; exec bash" --wait'
gnome-terminal --tab -e 'sh -c "sleep 14; rosrun execute Arm_control_real.py; exec bash" '
gnome-terminal --tab -e 'sh -c "sleep 14;  rosrun execute Arm_moveit.py; exec bash" '