#!/bin/sh
cpufreq-set -g performance
spread -c amirospread &
sleep 5
./senseHokuyo -o /lidar &
./senseOdometry -o /odom &
./senseCamJpg -o /cam -d6 &
./senseRingProximity -o /prox --noObstacleValues --noEdgeValues &
wait
cpufreq-set -g ondemand
