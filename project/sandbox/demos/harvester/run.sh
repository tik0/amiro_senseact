#!/bin/sh

if [ -z "${1}" ]; then
  echo "Set robot ID for the AMiRo"
  exit 1
fi


if [ -z "${2}" ]; then
  echo "Set marker ID for the AMiRo"
  exit 1
fi

ID=${1}
MARKER=${2}

STARTPOSX=-1.9
STARTPOSY=-1.55
STARTPOST=0.0

if [ ! -z "${3}" ]; then
  STARTPOSX=${3}
  if [ ! -z "${4}" ]; then
    STARTPOSY=${4}
    if [ ! -z "${5}" ]; then
      STARTPOST=${5}
    fi
  fi
fi

echo "Start AMiRo with ID ${ID} and tracking marker ${MARKER}"
echo " -> start position: ${STARTPOSX}/${STARTPOSY} m, ${STARTPOST}°"

./stop.sh
sleep 1

cpufreq-set -g performance

spread &
sleep 1
spread -c amirospread &

sleep 5

./setLights > /dev/null &
./harvester -a ${ID} -c harvest.xml --choreoDelay 5000 --idDelay 4000 -po --useTwb -m ${MARKER} --mmp 1.0 --startX ${STARTPOSX} --startY ${STARTPOSY} --startTheta ${STARTPOST} &

#wait
#cpufreq-set -g ondemand



