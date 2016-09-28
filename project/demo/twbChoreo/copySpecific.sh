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

# Copy specific run script
sshpass -p ${PASS} scp runAMiRo.sh root@${IP}:~/${NAME}

echo "Copying finished"
