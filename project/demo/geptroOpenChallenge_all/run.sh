#!/bin/sh

if [ -z "${1}" ]; then
  echo "Set a robot ID for the AMiRo"
  exit 1
fi

if [ -z "${2}" ]; then
  echo "Set a tracking ID for the AMiRo"
  exit 1
fi

robotID=${1}
trackingID=${2}
host=localhost
port=4823
prox_obstacle=/rir_prox/obstacle/

# kill maybe already started programs
./stop.sh
sleep 1

cpufreq-set -g performance

# start spread
spread -c amirospread &
sleep 5
spread &
sleep 5

# start all sensing programs
./objectSavingAMiRo -d 6 -s --loadingDirectly &
sleep 80
./rirReader -l > /dev/null &
./senseFloorProximity &

# start localization programs
./mapGenerator -r --id $trackingID --host $host --port $port --irin $prox_obstacle -l finalmap1.jpg -e finalmap_edge1.jpg &
./frontierExploration --id $trackingID --host $host --port $port --irin $prox_obstacle &

# start all primary moving programs
./motorControl > /dev/null &

# start all secondary moving programs
./localPlannerISY --id $trackingID --host $host --port $port &

# start all thrid level moving programs
./drivingObjectDetection --useTrackingData --trackingID $trackingID --meterPerPixel 0.0025 --trackingInscope /murox/roboterlocation --pathOutScope /path --pathResponseInscope /pathResponse --mapServerScope /mapGenerator&
./objectDelivery --host $host --port $port

# start only listening statemachines
./answerer --skipExploration --skipDetection --skipBlobbing --skipLocalPlanner --skipDelivery &
./openChallengeGEPTRO_2nd --robotID $robotID &

sleep 1

# start commanding statemachine
./answerer_tobi --robotID $robotID &

wait
cpufreq-set -g ondemand


