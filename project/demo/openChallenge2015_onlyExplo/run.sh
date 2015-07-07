#!/bin/sh

if [ -z "${1}" ]; then
  echo "Set an ID for the AMiRo"
  exit 1
fi

ID=${1}

# kill maybe already started programs
./stop.sh
sleep 1

cpufreq-set -g performance

# start spread
spread -c amirospread &
sleep 5

# start all sensing programs
./senseHokuyo -d /dev/ttyACM0 -o /lidar --publishNthScan 1 > /dev/null &
./senseOdometry -o /odom > /dev/null &
./rirReader -l > /dev/null &

# start localization programs
# ./CoreSLAM --lidarinscope /lidar --localizationOutScope /localization --serverScope /CoreSlamServer --mapServerReq map --remoteHost localhost --remotePort 4823 --senImage 0 --delta 0.01 --hole_width 0.05 &
# FAULTY # ./CoreSLAM --lidarinscope /lidar --localizationOutScope /localization --serverScope /CoreSlamServer --mapServerReq map --remoteHost localhost --remotePort 4823 --senImage 0 --delta 0.005 --hole_width 0.02 --mapAsImageOutScope /image &
./CoreSLAM --lidarinscope /lidar --localizationOutScope /localization --serverScope /CoreSlamServer --mapServerReq map --remoteHost localhost --remotePort 4823 --senImage 0 --delta 0.01 --hole_width 0.05 --mapAsImageOutScope /image &

# start all primary moving programs
./localPlanner -p /localization -i /path -r /pathResponse &
./exploDriveEdge -l > /dev/null &

# start all secondary moving programs
./drivingObjectDetection --skipPathPlanner --skipLocalPlanner --skipDetection &

# start only listening statemachines
./answerer --skipExploration --skipDetection > /dev/null &
./openChallengeGEPTRO_2nd --robotID ${ID} --testWithAnswerer &

sleep 1

# start commanding statemachine
./answerer_tobi --robotID ${ID} &

wait
cpufreq-set -g ondemand

