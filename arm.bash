#!/bin/bash

gnome-terminal --tab -e 'sh -c "source ./devel/setup.bash; roslaunch arm_moveit demo.launch; exec bash" --wait'
gnome-terminal --tab -e 'sh -c "sleep 14; source devel/setup.bash; rosrun execute Arm_control_real; exec bash" '
gnome-terminal --tab -e 'sh -c "sleep 14; source ./devel/setup.bash; rosrun execute Arm_moveit; exec bash" '