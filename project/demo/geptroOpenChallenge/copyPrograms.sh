#!/bin/sh

IP=${1}
NAME=`basename "$PWD"`
ssh root@${IP} "mkdir -p ~/${NAME}"

for line in `ls -d */ | sed 's#\ ##g' | sed 's#\/##g' | grep -v CMakeFiles`; do
  scp ${line}/${line} root@${IP}:~/${NAME}
done

scp run.sh root@${IP}:~/${NAME}
scp stop.sh root@${IP}:~/${NAME}
# Copy config
scp amirospread root@${IP}:~/${NAME}
scp rsb.conf root@${IP}:~/${NAME}

scp -r ${MUROX_PROJECT}/process/objectDetection/objectSavingAMiRo/objectPics root@${IP}:~/${NAME}
scp maps/* root@${IP}:~/${NAME}
