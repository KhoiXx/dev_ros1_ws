#!/bin/bash

gnome-terminal --title "IMU" --tab -e 'sh -c "cd /home/khoixx/dev_ros1_ws; roslaunch robot_bringup robot_imu.launch; exec bash"'
gnome-terminal --title "THESIS_ROBOT" --tab -e 'sh -c "sleep 5; roslaunch robot_bringup robot_keyboard.launch; exec bash"'
gnome-terminal --title "CAMERA" --tab -e 'sh -c "sleep 8; roslaunch robot_bringup robot_with_camera.launch; exec bash"'
# gnome-terminal --title "RTABMAP" --tab -e 'sh -c "sleep 14; roslaunch rtabmap_ros rtabmap.launch   depth_topic:=/camera/aligned_depth_to_color/image_raw     rgb_topic:=/camera/color/image_raw     camera_info_topic:=/camera/color/camera_info  rviz:=true rtabmapviz:=false Grid/MaxGroundHeight:=0.3    Grid/NormalSegmentation:=false odom_frame_id:=odom visual_odometry:=false; exec bash"'
# gnome-terminal --title "CAMERA" --tab -e 'sh -c "sleep 20; rqt_graph; exec bash"'
gnome-terminal --title "RTABMAP" --tab -e 'sh -c "sleep 14; roslaunch robot_bringup robot_rtabmap.launch; exec bash"'


