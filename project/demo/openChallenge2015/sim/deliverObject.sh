#!/bin/sh

if [ -z "${1}" ]; then
  echo "Set an ID for the robot"
  exit 1
fi

if [ -z "${2}" ]; then
  echo "Set an ID for the object"
  exit 1
fi

Oid=${2}
Rid=${1}

rsb-sendcpp0.11 /tobiamiro${Rid} object${Oid}.txt

