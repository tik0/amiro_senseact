#!/bin/sh

./stop.sh
sleep 1

cpufreq-set -g performance

./senseFloorProximity -o /prox/floor > /dev/null &
./senseSimple3DTo2D -d 1000 -o /lidar -c > /dev/null &

sleep 1

./objectFetchTask -p /prox/floor -l /lidar -r 0.06 -a 0.03 -d

wait
cpufreq-set -g ondemand



