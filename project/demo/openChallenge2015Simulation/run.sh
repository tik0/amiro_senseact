#!/bin/sh
ID=${1}

./stop.sh

cpufreq-set -g performance

spread -c amirospread &
sleep 5

./answerer >> /dev/null &
./openChallengeGEPTRO_2nd --robotID ${ID} --testWithAnswerer >> /dev/null &
sleep 1
./answerer_tobi --robotID ${ID} &

wait
cpufreq-set -g ondemand

