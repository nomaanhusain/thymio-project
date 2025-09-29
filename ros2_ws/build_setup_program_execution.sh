#!/bin/bash
cd ~
source setup-ros2-discovery.sh
rm -r ros_logs/*
cd ros2_ws
rm *log*.csv
colcon build
source install/setup.bash

