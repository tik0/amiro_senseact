#!/bin/sh

./stop.sh
sleep 1

cpufreq-set -g performance

./senseFloorProximity -o /prox/floor > /dev/null &
sleep 1
./objectFetchTask --useFakeObject -p /prox/floor

wait
cpufreq-set -g ondemand



