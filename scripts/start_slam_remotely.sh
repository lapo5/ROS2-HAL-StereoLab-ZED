#!/bin/bash

SCRIPT="source /opt/ros/eloquent/setup.bash; source /home/roxy-jetson/ros2_ws/install/setup.bash; ros2 launch zed_camera zed.launch.py"
HOST="192.168.1.40"
USERNAME="roxy"
PASSWORD="roxy"
SCR=${SCRIPT/PASSWORD/${PASSWORD}}
sshpass -p ${PASSWORD} ssh -l ${USERNAME} ${HOST} "${SCRIPT}"
