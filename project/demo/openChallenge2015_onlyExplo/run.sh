#!/bin/sh
ID=${1}

./stop.sh

cpufreq-set -g performance

spread -c amirospread &
sleep 5

./senseHokuyo -d /dev/ttyACM0 -o /lidar --publishNthScan 3 > /dev/null &
./senseOdometry -o /odom >> /dev/null &
./rirReader -l >> /dev/null &
./exploDriveEdge -l >> /dev/null &
./CoreSLAM --lidarinscope /lidar --odominscope /odom --serverScope /CoreSlamServer --mapServerReq map --remoteHost localhost --remotePort 4823 --senImage 0
./answerer --skipExploration >> /dev/null &
./openChallengeGEPTRO_2nd --robotID ${ID} --testWithAnswerer &
sleep 1
./answerer_tobi --robotID ${ID} &

wait
cpufreq-set -g ondemand

