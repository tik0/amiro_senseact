#!/bin/sh

if [ -z "${1}" ]; then
  ID=""
else
  ID=${1}
fi

export RSB_TRANSPORT_SPREAD_PORT=4803
export RSB_TRANSPORT_SPREAD_HOST=192.168.102.$1

if [ "${ID}" -eq "${ID}" ] > /dev/null ; then
  rsb-sendcpp0.13 /homing ./states/abort.txt
  echo "Send 'abort' to AMiRo IP: ${ID}"
fi
