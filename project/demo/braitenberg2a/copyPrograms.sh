#!/bin/bash

for line in `ls -d */ | sed 's#\ ##g' | sed 's#\/##g' | grep -v CMakeFiles`; do
  scp ${line}/${line} root@192.168.1.1:~
done
scp run.sh root@192.168.1.1:~
scp stop.sh root@192.168.1.1:~
