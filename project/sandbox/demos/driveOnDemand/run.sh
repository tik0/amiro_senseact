#!/bin/sh

./stop.sh
sleep 1

cpufreq-set -g performance

spread &
spread -c amirospread &

sleep 5

./setLights > /dev/null &

./driveOnDemand --port 4823 --host localhost &

#wait
#cpufreq-set -g ondemand



