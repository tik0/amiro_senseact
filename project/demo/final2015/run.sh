#!/bin/sh

if [ -z "${1}" ]; then
  echo "Set an ID for the AMiRo"
  exit 1
fi

ID=${1}

echo "Final setup: ToBI <- AMiRo ID 1 <- AMiRo ID 0"
echo "Start AMiRo with ID ${ID}"

sleep 2

./stop.sh

cpufreq-set -g performance

spread -c amirospread &

sleep 5

# sensing lidar from 'project/sense/senseHokuyo/'
./senseHokuyo -d /dev/ttyACM0 -o /AMiRo_Hokuyo/lidar &

# following from 'project/process/followToBI/'
./followToBI --lidarinscope /AMiRo_Hokuyo/lidar &

# waypoint program from 'project/sandbox/waypoint/'
./waypoint --lidarinscope /AMiRo_Hokuyo/lidar &

# start state machine
./final2015 --id ${ID} --turnAfterFollow 90 &
#--moveAfterFollow -300 &

wait
cpufreq-set -g ondemand
