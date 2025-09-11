#!/bin/bash
export MQTT_WS=~/FEI/10_semestre/Microcontroladores/projeto/ros2_ws
export ROS_DOMAIN_ID=13

LOG_FILE=/home/robot/log.txt
trap 'echo "Encerrando, limpando log..."; > "$LOGFILE"' EXIT

while ! nmcli -t -f DEVICE,STATE dev status | grep -q "wlp2s0:connected"; do
    echo "Aguardando hotspot subir..."
    sleep 2
done

source /opt/ros/humble/setup.bash
source $MQTT_WS/install/setup.bash

ros2 launch ros2_mqtt ros2_mqtt.launch.py        
