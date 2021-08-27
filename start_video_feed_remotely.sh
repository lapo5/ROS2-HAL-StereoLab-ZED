#!/bin/bash

SCRIPT="source /opt/ros/eloquent/setup.bash; source /home/roxy-jetson/ros2_ws/install/setup.bash; ros2 run zed_camera camera_node"
HOST="192.168.1.40"
USERNAME="roxy-jetson"
PASSWORD="roxy"
SCR=${SCRIPT/PASSWORD/${PASSWORD}}
sshpass -p ${PASSWORD} ssh -l ${USERNAME} ${HOST} "${SCRIPT}"