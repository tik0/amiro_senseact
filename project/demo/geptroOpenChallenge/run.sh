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
# TODO start sensing odometry!
./rirReader -l > /dev/null &

# start localization programs
# TODO start map builder
# TODO start exploration program

# start all primary moving programs
# start Local Planner

# start all secondary moving programs
#./drivingObjectDetection --skipPathPlanner --skipLocalPlanner --skipDetection &

# start only listening statemachines
./answerer &
./openChallengeGEPTRO_2nd --robotID ${ID} --testWithAnswerer &

sleep 1

# start commanding statemachine
./answerer_tobi --robotID ${ID} &

wait
cpufreq-set -g ondemand

