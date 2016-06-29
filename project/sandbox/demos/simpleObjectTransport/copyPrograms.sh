#!/bin/bash

IP=${1}
NAME=`basename "$PWD"`

echo "Please enter password:"
read -s PASS

result=$(sshpass -p ${PASS} ssh root@${IP} "mkdir -p ~/${NAME}" 2>&1)
if [ "${result}" ]; then
  echo "Could not connect: ${result}"
  exit 1
fi


echo "Start copying"

for line in `ls -d */ | sed 's#\ ##g' | sed 's#\/##g' | grep -v CMakeFiles`; do
  sshpass -p ${PASS} scp ${line}/${line} root@${IP}:~/${NAME}
done

# Copy run and stop scripts
sshpass -p ${PASS} scp run.sh root@${IP}:~/${NAME}
sshpass -p ${PASS} scp stop.sh root@${IP}:~/${NAME}

# Copy RSB configs
sshpass -p ${PASS} scp ${MUROX_INCLUDE_DIRS}/extspread/amirospread root@${IP}:~/${NAME}
sshpass -p ${PASS} scp rsb.conf root@${IP}:~/${NAME}

# Copy commands
sshpass -p ${PASS} scp -r Commands root@${IP}:~/${NAME}

echo "Copying finished"
