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

# start all sensing programs
#./objectSavingAMiRo -d 6 -s --loadingDirectly &
./rirReader -l > /dev/null &
./senseFloorProximity &

# start localization programs
./mapGenerator -r --id $id --host $host --port $port --irin $prox_obstacle &
./frontierExploration --id $id --host $host --port $port --irin $prox_obstacle &

# start all primary moving programs
./motorControl > /dev/null &

# start all secondary moving programs
#./edgeAvoidanceBehavior > /dev/null &
./localPlannerISY --id $id --host $host --port $port &

# start all thrid level moving programs
./drivingObjectDetection --useTrackingData --trackingID ${trackingID} --meterPerPixel 0.0025 --trackingInscope /murox/roboterlocation --pathOutScope /path --pathResponseInscope /pathResponse &

# start only listening statemachines
./answerer --skipExploration --skipDetection &
./openChallengeGEPTRO_2nd --robotID ${robotID} --testWithAnswerer &

sleep 1

# start commanding statemachine
./answerer_tobi --robotID ${robotID} &

# just to start exploration
#sleep 2
#./rsbsend.sh /exploration start

wait
cpufreq-set -g ondemand



