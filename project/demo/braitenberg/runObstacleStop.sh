#!/bin/sh

./stop.sh
sleep 1

cpufreq-set -g performance

./senseRingProximity > /dev/null &
sleep 1
# braitenberg behavior with just turning until front is clear
./braitenberg -c --stopObstacle -o 0.17 -s 5 &
# braitenberg behavior with "overturning"
#./braitenberg -c --stopObstacle -o 0.17 -s 5 --overturn &
./frontObjectDetection -s &

wait
cpufreq-set -g ondemand



