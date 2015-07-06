#!/bin/sh
ID=${1}

# kill maybe already started programs
./stop.sh
sleep 1

cpufreq-set -g performance

# start spread
spread -c amirospread &
sleep 5

# start all sensing programs
./senseHokuyo -d /dev/ttyACM0 -o /lidar --publishNthScan 3 > /dev/null &
./senseOdometry -o /odom > /dev/null &
./rirReader -l > /dev/null &

# start localization programs
./CoreSLAM --lidarinscope /lidar --localizationOutScope /localization --serverScope /CoreSlamServer --mapServerReq map --remoteHost localhost --remotePort 4823 --senImage 0 &

# start all moving programs
./exploDriveEdge -l > /dev/null &
./drivingObjectDetection --skipPathPlanner --skipLocalPlanner --skipDetection > /dev/null &

# start only listening statemachines
./answerer --skipExploration --skipDetection > /dev/null &
./openChallengeGEPTRO_2nd --robotID ${ID} --testWithAnswerer &

# start commanding statemachine
sleep 1
./answerer_tobi --robotID ${ID} &

wait
cpufreq-set -g ondemand

