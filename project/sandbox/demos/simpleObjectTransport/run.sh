#!/bin/sh

if [ -z "${1}" ]; then
  echo "Set an ID for the AMiRo"
  exit 1
fi

ID=${1}

echo "Start AMiRo with ID ${ID}:"

sleep 1

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
# --id: robot's id (default: 0).
# -f: Forward speed in m/s (default: 0.06).
# -t: Angular speed in degree/s (default: 20.0).
# -i: Maximal distance for command detection by the proximity sensors in m (default: 0.05).
# -d: Distance for edge detection in m (default: 0.055).
# -v: Maximal variance between the proximity sensors for edge orientation in m (default: 0.005).
# -e: Distance between robot and table edge for setting and grasping objects onto the robot  in m (default: 0.05).
# -b: Buffer between command recognition and start in seconds (default: 5).
# -r: Flag, if the commands shall only be given via RSB (otherwise they can also be given with the proximity sensor ring).
./objectTransport --id ${ID} -f 0.08 -t 20.0 -i 0.01 -d 0.055 -v 0.008 -e 0.1 -b 5 &

wait
cpufreq-set -g ondemand



