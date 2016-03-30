#!/bin/sh

IP=${1}
NAME=`basename "$PWD"`
ssh root@${IP} "mkdir -p ~/${NAME}"

for line in `ls -d */ | sed 's#\ ##g' | sed 's#\/##g' | grep -v CMakeFiles`; do
  scp ${line}/${line} root@${IP}:~/${NAME}
done
scp background.sh root@${IP}:~/${NAME}
scp backgroundStop.sh root@${IP}:~/${NAME}
scp run.sh root@${IP}:~/${NAME}
scp stop.sh root@${IP}:~/${NAME}
# Copy config
scp ${MUROX_INCLUDE_DIRS}/conf/RoboCup2016/spread/amirospread root@${IP}:~/${NAME}
scp rsb.conf root@${IP}:~/${NAME}

scp -r sim root@${IP}:~/${NAME}/
scp -r objectPics root@${IP}:~/${NAME}/
