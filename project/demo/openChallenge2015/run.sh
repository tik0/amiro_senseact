#!/bin/sh
./kill.sh
cpufreq-set -g perfomance
spread -c amirospread &
sleep 5

./senseHokuyo -d /dev/ttyACM0 -o /lidar &
./senseOdometry -o /odom &
./senseRingProximity -o /prox &
./actAmiroMotor -i /motor &
# TODO ./exploDriveEdge ?? &
# TODO ./CoreSLAM ?? &
./openChallenge2015 -i /tobiamiro1 --outscopeStateTobi /amiro1tobi &
wait
cpufreq-set -g ondemand

