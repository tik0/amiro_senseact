#!/bin/sh
./stop.sh

# sensing lidar from 'project/sense/senseHokuyo/'
./senseHokuyo -d /dev/ttyACM0 -o /AMiRo_Hokuyo/lidar &
# following from 'project/process/followToBI/'
./followToBI -s &
