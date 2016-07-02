#!/bin/sh

./stop.sh
sleep 1

cpufreq-set -g performance

spread &
spread -c amirospread &

sleep 5

./setLights > /dev/null &

./driveOnRRL --port 4823 --host localhost --drivingdist 0.1 &

#wait
#cpufreq-set -g ondemand



