#!/bin/sh

if [ -z "${1}" ]; then
  echo "Set a robot ID for the AMiRo."
  exit 1
fi
robotID=${1}

if [ -z "${2}" ]; then
  echo "Set a tracking ID for the AMiRo."
  exit 1
fi
trackingID=${2}

if [ -z "${3}" ]; then
  echo "Setting the destination ID to the tracking ID of the AMiRo (simply setting delivering destination to start position)."
  destinationID=$trackingID
else
  destinationID=${3}
fi

host=localhost
port=4823
prox_obstacle=/rir_prox/obstacle/

# kill maybe already started programs
./stop.sh
sleep 1

cpufreq-set -g performance

# start all sensing programs
./rirReader > /dev/null &
./senseFloorProximity &

# start localization programs
./mapGenerator -r --id $trackingID --host $host --port $port --irin $prox_obstacle &
#./mapGenerator -r --id $trackingID --host $host --port $port --irin $prox_obstacle -l finalmap1.jpg -e finalmap_edge1.jpg &
./frontierExploration --id $trackingID --host $host --port $port --irin $prox_obstacle &

# start all primary moving programs
./motorControl > /dev/null &

# start all secondary moving programs
./localPlannerISY --id $trackingID --host $host --port $port &

# start all thrid level moving programs
./drivingObjectDetection --useTrackingData --trackingID $trackingID --meterPerPixel 0.0025 --trackingInscope /murox/roboterlocation --pathOutScope /path --pathResponseInscope /pathResponse --mapServerScope /mapGenerator&
./objectDelivery --host $host --port $port --destId $destinationID &

# start only listening statemachines
./amiroEnvironment --skipDetection --skipBlobbing --skipLocalPlanner --skipDelivery &
./stateMachineGEPTRO --robotID $robotID &

sleep 1

# start commanding statemachine
./answerer_tobi --robotID $robotID &

wait
cpufreq-set -g ondemand



