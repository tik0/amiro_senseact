#!/bin/bash
./stop.sh
# Start the spread communication deamon
spread -c spreadSim.conf &
sleep 5
# Start the simulator
gazebo table3.world > /dev/null &
# Start the SLAM program
./mapBuilder/mapBuilder --odominscope /AMiRo_Hokuyo/gps/ --lidarinscope /AMiRo_Hokuyo/lidar/ --senImage 1 --delta 0.01  -s true &
./localPlanner/localPlanner -s --steeringOut /AMiRo_Hokuyo/diffKin > debug.txt &
./frontierExploration/frontierExploration &
sleep 3
rsb-sendcpp0.11 /exploration run.sh
