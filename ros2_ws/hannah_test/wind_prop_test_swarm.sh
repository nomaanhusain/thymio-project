#!/bin/bash

CSV_FILE="ips_swarm.csv"
USERNAME="thymio"
PASSWORD="thymio"

while IFS=',' read -r ip; do
    echo "Connecting to $ip to run publish_wind_direction"
    sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no "$USERNAME@$ip" << EOF
        cd ~/ros2_ws/wind_properties_test/
        nohup python3 main.py > random.log 2>&1 &
        echo "Started python program on $ip"
EOF
done < "$CSV_FILE"
