#!/bin/bash
./stopWaypoint.sh
# Start the spread communication deamon
spread  &
sleep 5
# Start the SLAM program
./senseHokuyo -o /scan&
./waypoint --lidarinscope /scan &
sleep 3
rsb-sendcpp0.11 /waypoint/command init.txt
echo "Init waypoint"
