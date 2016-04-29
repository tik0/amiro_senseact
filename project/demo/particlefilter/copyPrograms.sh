#!/bin/sh

IP=${1}
NAME=`basename "$PWD"`
ssh root@${IP} "mkdir -p ~/${NAME}"

files=""

for line in `ls -d */ | sed 's#\ ##g' | sed 's#\/##g' | grep -v CMakeFiles`; do
  files="$files ${line}/${line}"
done

files="run.sh $files"
files="stop.sh $files"

# Copy RSB config
files="rsb.conf $files"

# Copy the map
files="${MUROX_PROJECT}/demo/CoreSLAMLocalization/data/centralLab-clean-cropped-valid-4-scale-0.5.png $files"

scp $files root@${IP}:~/${NAME}
