#!/bin/sh
cpufreq-set -g performance
spread -c amirospread &
sleep 5
./senseHokuyo -o /lidar &
./sendOdometryProtoPose -o /odom &
./senseCamJpg -o /cam -d6 &
./senseRingProximity -o /prox --noObstacleValues --noEdgeValues &
./senseFloorProximity -o /proxFloor
wait
cpufreq-set -g ondemand
