#!/bin/sh

./stop.sh
sleep 1

cpufreq-set -g performance

./senseFloorProximity -o /prox/floor > /dev/null &

# Sensing the laser data of the RGBD camera:
# -c: Just take the vertical centered laser scan instead of the minimum 
./senseSimple3DTo2D -d 1000 -o /lidar -c > /dev/null &

sleep 1

# Main task:
# -p: Scope for receiving floor proximity sensor values
# -l: Scope for receiving scanner values (independed if laser scan is from laser scanner or RGBD cam)
# -r: Object radius in m
# -a: Raduis addition in m to the robot's radius, which is 0.05 m
# -d: Flag, that a RGBD camera is used instead of a "real" laser scanner
# -f: Minimal sensor value of the floor proximity sensors for detecting the table
./objectFetchTask -p /prox/floor -l /lidar -r 0.06 -a 0.03 -d -f 15000

wait
cpufreq-set -g ondemand



