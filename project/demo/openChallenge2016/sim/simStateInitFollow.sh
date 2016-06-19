#!/bin/sh

if [ -z "${1}" ]; then
  ID=""
else
  ID=${1}
fi

if [ "${ID}" -eq "${ID}" ] > /dev/null ; then
  rsb-sendcpp0.13 /tobiamiro${ID}/state states/initFollow.txt
  echo "Send 'initFollow' to AMiRo ${ID}"
else
  for num in `seq 0 9` ; do
    rsb-sendcpp0.13 /tobiamiro${num}/state states/initFollow.txt
    echo "Send 'initFollow' to AMiRo ${num}"
  done
fi
