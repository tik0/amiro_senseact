#!/bin/bash
./stop.sh
# Start the spread communication deamon
spread -c spreadSim.conf &
sleep 5
# Start the simulator
gazebo table0.world > /dev/null &
# Start the SLAM program
./mapBuilder/mapBuilder --odominscope /AMiRo_Hokuyo_tilted/gps/ --lidarinscope /AMiRo_Hokuyo_tilted/lidar/ --senImage 1 --delta 0.01 --rotY 30 &
sleep 3
./localPlanner/localPlanner -s --steeringOut /AMiRo_Hokuyo_tilted/diffKin > debug.txt &
./frontierExploration/frontierExploration &
