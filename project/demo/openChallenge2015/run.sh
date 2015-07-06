#!/bin/sh

if [ -z "${1}" ]; then
  echo "Set an ID for the AMiRo"
  exit 1
fi

./stop.sh


cpufreq-set -g performance
spread -c amirospread &
sleep 5

./senseHokuyo -d /dev/ttyACM0 -o /lidar --publishNthScan 3 > /dev/null &
./senseOdometry -o /odom &
#./senseRingProximity -o /prox &
#./actAmiroMotor -i /motor &
# TODO ./exploDriveEdge ?? &
./CoreSLAM --lidarinscope /lidar --localizationOutScope /localization --serverScope /CoreSlamServer --mapServerReq map --remoteHost localhost --remotePort 4823 --senImage 0
#./openChallenge2015 -i /tobiamiro1 --outscopeStateTobi /amiro1tobi &
wait
cpufreq-set -g ondemand

