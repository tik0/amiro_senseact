#!/bin/sh

./stop.sh
sleep 1

cpufreq-set -g performance

./rirReader > /dev/null &
sleep 1
./following

wait
cpufreq-set -g ondemand



