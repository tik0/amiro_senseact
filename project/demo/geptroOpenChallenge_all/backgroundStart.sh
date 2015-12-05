#!/bin/sh

# kill maybe already started programs
./allStop.sh
sleep 1

#cpufreq-set -g performance

# start spread
spread -c amirospread &
sleep 5
spread &
sleep 5

# start all loading programs
./objectDetection -d 6 -s --loadingDirectly &

#wait
#cpufreq-set -g ondemand



