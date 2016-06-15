#!/bin/sh

./stop.sh
sleep 1

cpufreq-set -g performance

spread &
spread -c amirospread &

sleep 5

./senseRingProximity -o /rir_prox/obstacle > /dev/null &
./setLights > /dev/null &

sleep 1

# Transport
# -f: Forward speed in m/s (default: 0.08).
# -t: Angular speed in degree/s (default: 20.0).
# -i: Maximal distance for command detection by the proximity sensors in m (default: 0.05).
# -d: Distance for edge detection in m (default: 0.055).
# -v: Maximal variance between the proximity sensors for edge orientation in m (default: 0.005).
# -e: Distance between robot and table edge for setting and grasping objects onto the robot  in m (default: 0.05).
./objectTransport -f 0.08 -t 20.0 -i 0.05 -d 0.055 -v 0.006 -e 0.05

wait
cpufreq-set -g ondemand



