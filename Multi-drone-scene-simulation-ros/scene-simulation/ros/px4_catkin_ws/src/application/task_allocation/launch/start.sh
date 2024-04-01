#!/bin/bash
gnome-terminal --window -e 'bash -c "roslaunch uav_control uavs5_highway.launch; exec bash"' \
--tab -e 'bash -c "echo \"task_ loading\"; read; rosrun task_generation task_generation; exec bash"' \
--tab -e 'bash -c "echo \"view_point_plan\"; read; rosrun view_point_plan cal_path.py; exec bash"' \
--tab -e 'bash -c "echo \"task_management\"; read; rosrun task_management task_management; exec bash"' \
--tab -e 'bash -c "echo \"center_allocate\"; read; rosrun task_allocation center_allocate.py; exec bash"' \
--tab -e 'bash -c "echo \"uavs_bid\"; read; roslaunch task_allocation uavs5_bid.launch; exec bash"' \
--tab -e 'bash -c "echo \"uav_control\"; read; roslaunch uav_control uavs5.launch; exec bash"' \
