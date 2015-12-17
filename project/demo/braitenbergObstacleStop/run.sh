#!/bin/sh

./stop.sh
sleep 1

cpufreq-set -g performance

./senseRingProximity > /dev/null &
sleep 1
./braitenbergObstacleStop -c -o 0.17 -s 5 &
./frontObjectDetection -sd 6 &

wait
cpufreq-set -g ondemand



