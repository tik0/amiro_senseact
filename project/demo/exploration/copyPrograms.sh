#!/bin/sh

IP=${1}
NAME=`basename "$PWD"`
ssh root@${IP} "mkdir -p ~/${NAME}"

for line in `ls -d */ | sed 's#\ ##g' | sed 's#\/##g' | grep -v CMakeFiles`; do
  scp ${line}/${line} root@${IP}:~/${NAME}
done
scp *.sh root@${IP}:~/${NAME}
scp amirospread root@${IP}:~/${NAME}
