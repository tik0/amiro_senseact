#!/bin/sh

if [ -z "${1}" ]; then
  echo "Set a file to send ('sim/song[0/1/2/4/5]')."
  exit 1
fi
songname=${1}

rsb-sendcpp0.11 /choreo $songname

