#!/bin/sh
ID=${1}
Echo "Final setup: Tobi <- AMiRo ID 1 <- AMiRo ID 0"
echo "Start AMiRo with ID ${ID}"
./stop.sh
cpufreq-set -g performance
spread -c amirospread &
sleep 5

# sensing lidar from 'project/sense/senseHokuyo/'
./senseHokuyo -d /dev/ttyACM0 -o /AMiRo_Hokuyo/lidar &
sleep 3
# following from 'project/process/followToBI/'
./followToBI &
# waypoint program from 'project/sandbox/waypoint/'
./waypoint --lidarinscope /AMiRo_Hokuyo/lidar &

# start state machine
./final2015 --id ${ID} &

wait
cpufreq-set -g ondemand
