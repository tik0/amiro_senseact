#!/bin/sh

./stop.sh
sleep 1

cpufreq-set -g performance

spread -c amirospread &

sleep 5

./driveOnDemand &

#wait
#cpufreq-set -g ondemand



