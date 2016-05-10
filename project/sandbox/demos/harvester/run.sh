#!/bin/sh

if [ -z "${1}" ]; then
  echo "Set an ID for the AMiRo"
  exit 1
fi

ID=${1}

echo "Start AMiRo with ID ${ID}"

./stop.sh
sleep 1

cpufreq-set -g performance

spread &
sleep 1
spread -c amirospread &

sleep 5

./harvester -a ${ID} -c harvest.xml -rp &

#wait
#cpufreq-set -g ondemand



