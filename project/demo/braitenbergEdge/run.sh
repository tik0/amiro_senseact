#!/bin/sh

./stop.sh
sleep 1

cpufreq-set -g performance

./rirReader -l > /dev/null &
sleep 1
./braitenbergEdge -c

wait
cpufreq-set -g ondemand



