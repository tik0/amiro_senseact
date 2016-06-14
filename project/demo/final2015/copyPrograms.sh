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
scp ${MUROX_INCLUDE_DIRS}/extspread/amirospread root@${IP}:~/${NAME}
scp rsb.conf root@${IP}:~/${NAME}

# Copy simulation folder
scp -r sim root@${IP}:~/${NAME}/

# Copy the website
scp -r ${MUROX_PROJECT}/process/misc/rsb_ws_bridge_amiro/www root@${IP}:~/${NAME}
