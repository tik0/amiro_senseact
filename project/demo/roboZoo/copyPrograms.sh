#!/bin/bash

IP=${1}

for line in `ls -d */ | sed 's#\ ##g' | sed 's#\/##g' | grep -v CMakeFiles`; do
  scp ${line}/${line} root@${IP}:~
done
scp run.sh root@${IP}:~
scp stop.sh root@${IP}:~
# Copy scripts
scp Choreos/* root@${IP}:~
# Copy config
scp amirospread root@${IP}:~
