#!/bin/bash
export OMNICARE_WS=/home/robot/TCC/software
export ROS_DOMAIN_ID=13

LOG_FILE=/home/robot/log.txt
trap 'echo "Encerrando, limpando log..."; > "$LOGFILE"' EXIT

while ! nmcli -t -f DEVICE,STATE dev status | grep -q "wlP1p1s0:connected"; do
    echo "Aguardando hotspot subir..."
    sleep 2
done

source /opt/ros/humble/setup.bash
source $OMNICARE_WS/install/setup.bash

ros2 launch ros2_mqtt ros2_mqtt.launch.py        
