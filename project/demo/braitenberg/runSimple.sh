#!/bin/sh

./stop.sh
sleep 1

cpufreq-set -g performance

./senseRingProximity --noObstacleValues --noEdgeValues > /dev/null &

sleep 1

./braitenberg -c -f 2700 --overturn

wait
cpufreq-set -g ondemand



