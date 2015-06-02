#!/bin/bash
./killFinal.sh
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
#./final2015 &
