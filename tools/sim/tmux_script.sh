#!/bin/bash
tmux new -d -s carla-sim
tmux send-keys "./launch_openpilot.sh" ENTER

tmux neww
tmux send-keys "source /opt/ros/noetic/setup.bash ; . /opt/ros/noetic/setup.bash ;  ./hamidbridge.py --dual_camera --environment $BRIDGEENV  --rosip  $ROS_IP | tee  bridge-log.txt $*" ENTER

tmux neww
tmux send-keys "./../replay/ui.py $*" ENTER

tmux a -t carla-sim
