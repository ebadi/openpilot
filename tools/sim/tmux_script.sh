#!/bin/bash
tmux new -d -s carla-sim
tmux send-keys "./launch_openpilot.sh" ENTER


#tmux neww
#tmux send-keys "export ROS_MASTER_URI=http://192.168.0.200:11311 ; export ROS_IP=192.168.0.100 ; source /opt/ros/noetic/setup.bash ; . /opt/ros/noetic/setup.bash ; cd gokart_controllerx ; $*"  ENTER "python gokart.py" ENTER

tmux neww
tmux send-keys "./hamidbridge.py --dual_camera | tee | tee  bridge-log.txt $*" ENTER

tmux neww
tmux send-keys "./../replay/ui.py $*" ENTER

tmux a -t carla-sim
