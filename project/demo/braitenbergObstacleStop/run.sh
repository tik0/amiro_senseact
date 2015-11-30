#!/bin/sh

./stop.sh
sleep 1

cpufreq-set -g performance

./rirReader -l > /dev/null &
sleep 1
./braitenbergObstacleStop -c > /dev/null &
./frontObjectDetection -sd 6 > /dev/null &

wait
cpufreq-set -g ondemand



