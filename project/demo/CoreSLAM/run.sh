#!/bin/sh
cpufreq-set -g perfomance
spread &
sleep 5
./CoreSLAM --odominscope /odom --lidarinscope /lidar --senImage 1 --delta 0.01 --sigma_xy 0.01 &
./actAmiroMotor -i /motor &
./senseHokuyo -o /lidar &
./senseOdometry -o /odom &
wait
cpufreq-set -g ondemand
