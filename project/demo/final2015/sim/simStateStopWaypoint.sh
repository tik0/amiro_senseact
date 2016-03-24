#!/bin/sh

if [ -z "${1}" ]; then
  ID=""
else
  ID=${1}
fi

if [ "${ID}" -eq "${ID}" ] > /dev/null ; then
  rsb-sendcpp0.11 /tobiamiro${ID}/state states/stopWaypoint.txt
  echo "Send 'stopWaypoint' to AMiRo ${ID}"
else
  for num in `seq 0 9` ; do
    rsb-sendcpp0.11 /tobiamiro${num}/state states/stopWaypoint.txt
    echo "Send 'stopWaypoint' to AMiRo ${num}"
  done
fi
