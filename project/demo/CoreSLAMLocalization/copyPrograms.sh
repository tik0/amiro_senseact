#!/bin/bash

IP=${1}
if [ -z $IP ]; then
  echo 'no IP address given!' >&2
  exit 1
fi
NAME=`basename "$PWD"`
ssh root@$IP "mkdir -p ~/${NAME}"

# collect files
FILES="run.sh stop.sh amirospread rsb.conf"
for line in `ls -d */ | sed 's#\ ##g' | sed 's#\/##g' | grep -v CMakeFiles`; do
  FILES="$FILES ${line}/${line}"
done

# copy files
scp $FILES root@$IP:~/$NAME/
