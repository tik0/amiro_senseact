#!/bin/sh

if [ -z "${1}" ]; then
  echo "Set an ID for the AMiRo"
  exit 1
fi

ID=${1}

if [ "${ID}" -eq "${ID}" ] > /dev/null ; then
  rsb-sendcpp0.11 /tobiamiro${ID}/state states/initFollow.txt
  echo "Send 'initFollow' to AMiRo ${ID}"
else
  for num in `seq 0 9` ; do
    rsb-sendcpp0.11 /tobiamiro${num}/state states/initFollow.txt
    echo "Send 'initFollow' to AMiRo ${num}"
  done
fi
