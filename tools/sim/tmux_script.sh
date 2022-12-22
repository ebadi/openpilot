#!/bin/bash
tmux new -d -s carla-sim
tmux send-keys "./launch_openpilot.sh" ENTER

#tmux neww
#localipaddress=$(ifconfig | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1' | grep 192.168)
#export ROS_MASTER_URI=http://192.168.1.200:11311 ; export ROS_IP=$localipaddress ; source /opt/ros/noetic/setup.bash ; . /opt/ros/noetic/setup.bash ; rostopic pub Speed_Request_Turn std_msgs/Float32 0


tmux neww
localipaddress=$(ifconfig | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1' | grep 192.168)
tmux send-keys "export ROS_MASTER_URI=http://192.168.1.200:11311 ; export ROS_IP=$localipaddress ; source /opt/ros/noetic/setup.bash ; . /opt/ros/noetic/setup.bash ;  ./hamidbridge.py --dual_camera --environment carla| tee  bridge-log.txt $*" ENTER

tmux neww
tmux send-keys "./../replay/ui.py $*" ENTER

tmux a -t carla-sim
