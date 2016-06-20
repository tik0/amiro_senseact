#!/bin/sh

IP=${1}
NAME=`basename "$PWD"`
ssh root@${IP} "mkdir -p ~/${NAME}/data"

files=""
for line in `ls -d */ | sed 's#\ ##g' | sed 's#\/##g' | grep -v CMakeFiles`; do
	files="$files ${line}/${line}"
done
files="$files run.sh stop.sh"


# Copy config
files="$files ${MUROX_INCLUDE_DIRS}/extspread/amirospread"
files="$files rsb.conf"

# Copy simulation folder
files="$files sim"

# Copy the website
#files="$files ${MUROX_PROJECT}/process/misc/rsb_ws_bridge_amiro/www"

dest="root@${IP}:~/${NAME}/"
rsync -v --progress -ua $files $dest

# Copy the maps
rsync -v --progress -au ${MUROX_PROJECT}/demo/CoreSLAMLocalization/data/*.png "$dest/data/"
