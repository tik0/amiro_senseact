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

# kill maybe already started programs
./stop.sh
sleep 1

cpufreq-set -g performance

# start spread
spread -c amirospread &
sleep 5

# start all sensing programs
# TODO start sensing odometry!
./rirReader -l > /dev/null &

# start localization programs
# TODO start map builder
# TODO start exploration program

# start all primary moving programs
# start Local Planner

# start all secondary moving programs
./drivingObjectDetection --useTrackingData --trackingID ${trackingID} --meterPerPixel 0.0025 --trackingInscope /murox/roboterlocation --skipPathPlanner --skipLocalPlanner --skipFinalRotation --skipDetection --skipCorrection --skipLocalization &

# start only listening statemachines
./answerer --skipDetection &
./openChallengeGEPTRO_2nd --robotID ${robotID} --testWithAnswerer &

sleep 1

# start commanding statemachine
./answerer_tobi --robotID ${robotID} &

wait
cpufreq-set -g ondemand

