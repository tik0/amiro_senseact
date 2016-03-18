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

# start all sensing programs
./senseHokuyo -d /dev/ttyACM0 -o /lidar --publishNthScan 3 > /dev/null &
./sendOdometryProtoPose -o /odom &
./senseRingProximity > /dev/null &

# start localization programs
# ./CoreSLAM --lidarinscope /lidar --localizationOutScope /localization --serverScope /CoreSlamServer --mapServerReq map --remoteHost localhost --remotePort 4823 --senImage 0 --delta 0.01 --hole_width 0.05 &
# FAULTY # ./CoreSLAM --lidarinscope /lidar --localizationOutScope /localization --serverScope /CoreSlamServer --mapServerReq map --remoteHost localhost --remotePort 4823 --senImage 0 --delta 0.005 --hole_width 0.02 --mapAsImageOutScope /image &
./CoreSLAM --robotID ${ID} --lidarinscope /lidar --localizationOutScope /localization --serverScope /CoreSlamServer --mapServerReq map --remoteHost localhost --remotePort 4823 --senImage 0 --delta 0.005 --hole_width 0.1 --mapAsImageOutScope /image --samples 250 --throttle_scans 1 --sigma_xy 10.0 --sigma_theta 0.05 &

# start all primary moving programs
./localPlanner -p /localization -i /path -r /pathResponse > /dev/null &
./exploDriveEdge &

# start all secondary moving programs
./drivingObjectDetection --bigMap --mapServerIsRemote --mapServerScope /CoreSlamServer --pathRequest path --robotID ${ID} &
./objectDelivery --useSLAMData --bigMap --positionInscope /localization --pathOut /path --pathRe /pathResponse --mapServer /CoreSlamServer --mapServerIsRemote --robotID ${ID} &

# start only listening statemachines
./amiroEnvironment --skipExploration --skipDetection --skipDelivery &
./stateMachineGEPTRO --robotID ${ID} --bigMap --mapServerIsRemote --mapServerScope /CoreSlamServer --obstacleServerReq getObjectsList &

sleep 1

# start commanding statemachine (simulation)
./commandSimulation --robotID ${ID} &

wait
cpufreq-set -g ondemand

