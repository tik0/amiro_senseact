#!/bin/sh

IP=${1}
NAME=`basename "$PWD"`
ssh root@${IP} "mkdir -p ~/${NAME}"

for line in `ls -d */ | sed 's#\ ##g' | sed 's#\/##g' | grep -v CMakeFiles`; do
  scp ${line}/${line} root@${IP}:~/${NAME}
done

# Copy run and stop scripts
scp run.sh root@${IP}:~/${NAME}
scp stop.sh root@${IP}:~/${NAME}

# Copy RSB configs
scp amirospread root@${IP}:~/${NAME}
scp rsb.conf root@${IP}:~/${NAME}

# Copy commands
scp -r Commands root@${IP}:~/${NAME}
