#!/bin/sh

./stop.sh
sleep 1

cpufreq-set -g performance

./senseFloorProximity -o /prox/floor > /dev/null &
sleep 1
./objectFetchTask --useFakeObject -t 100 -p /prox/floor

wait
cpufreq-set -g ondemand



