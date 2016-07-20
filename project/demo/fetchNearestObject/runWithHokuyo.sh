#!/bin/sh

./stop.sh
sleep 1

cpufreq-set -g performance

./senseFloorProximity -o /prox/floor > /dev/null &
./senseHokuyo -d /dev/ttyACM0 -o /lidar > /dev/null &

sleep 1

# Main task:
# -p: Scope for receiving floor proximity sensor values
# -l: Scope for receiving scanner values (--anglePositiveToRight just corrects the angle direction of the scanner)
# -r: Object radius in m
# -w: Angle the laser scanner should only look at at its front (in degrees)
# -f: Minimal sensor value of the floor proximity sensors for detecting the table
./objectFetchTask -p /prox/floor -l /lidar --anglePositiveToRight -r 0.06 -w 30.0 -f 15000

wait
cpufreq-set -g ondemand



