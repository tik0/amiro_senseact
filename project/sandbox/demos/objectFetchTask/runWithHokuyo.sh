#!/bin/sh

./stop.sh
sleep 1

cpufreq-set -g performance

./senseFloorProximity -o /prox/floor > /dev/null &
./senseHokuyo -d /dev/ttyACM0 -o /lidar > /dev/null &

sleep 1

./objectFetchTask -p /prox/floor -l /lidar --anglePositiveToRight -r 0.06 -w 30.0

wait
cpufreq-set -g ondemand



