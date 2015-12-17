#!/bin/sh

./stop.sh
sleep 1

cpufreq-set -g performance

./senseRingProximity > /dev/null &
sleep 1
./braitenberg -c &
./frontObjectDetection -s &

wait
cpufreq-set -g ondemand



