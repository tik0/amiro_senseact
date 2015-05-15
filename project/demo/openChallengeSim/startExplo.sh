#!/bin/bash
./stopExplo.sh
# Start the spread communication deamon
#spread  &
#sleep 5
# Start the SLAM program
./senseHokuyo -o /scan&
./senseOdometry -o /odometry &
./mapBuilder --odominscope /odometry --lidarinscope /scan --senImage 1 --delta 0.01 --rotY 30  &
./localPlanner    &
./frontierExploration &
sleep 3
rsb-sendcpp0.11 /exploration startExplo.sh
echo "Starting Exploration"
