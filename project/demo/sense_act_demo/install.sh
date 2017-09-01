#!/bin/bash

set -e
source /opt/poky/1.7.2/environment-setup-cortexa8hf-vfp-neon-poky-linux-gnueabi
cmake -DCMAKE_BUILD_TYPE=Release .
make -j4
set +e

destination=ros_sense_act_tools
if [ -d "${destination}" ]; then
  rm -r ${destination}
fi
mkdir ${destination}
for line in $(ls -d */ | sed 's#\ ##g' | sed 's#\/##g' | grep -v CMakeFiles); do
  if [ ${line} == ${destination} ]; then
    continue
  fi
  cp ${line}/${line} ${destination}
done

cp rsb.conf ${destination}/
cp start.sh ${destination}/
cp stop.sh ${destination}/

echo "Done -- Copied files to ${destination}"
