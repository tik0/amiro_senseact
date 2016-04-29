#!/bin/sh

./stop.sh
sleep 1

cpufreq-set -g performance

./senseRingProximity > /dev/null &

sleep 1

./braitenberg -c --overturn &


wait
cpufreq-set -g ondemand



