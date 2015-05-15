#!/bin/bash
./stopFinal.sh
# Start the spread communication deamon
spread -c amirospread &
sleep 5
./senseHokuyo -o /scan > /dev/null &
./waypoint --lidarinscope /scan > /dev/null &
./followToBI -l /scan > /dev/null &
./final2015 --id 0 --turn -130 & > /dev/null &

