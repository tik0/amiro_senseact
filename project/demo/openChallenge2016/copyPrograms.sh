#!/bin/sh

set -e

PASS=$1
shift

for IP in $@; do
	echo "====================="
	echo "copying to ${IP}"
	echo "====================="

	#IP=${1}
	NAME=`basename "$PWD"`
	sshpass -p$PASS ssh root@${IP} "mkdir -p ~/${NAME}/data"

	files=""
	for line in `ls -d */ | sed 's#\ ##g' | sed 's#\/##g' | grep -v CMakeFiles`; do
		if [ -f "${line}/${line}" ]; then
			files="$files ${line}/${line}"
		else
			echo ${line}/${line} does not exist
		fi
	done
	files="$files run.sh stop.sh"


	# Copy config
	files="$files ${MUROX_INCLUDE_DIRS}/extspread/amirospread"
	files="$files rsb.conf"

	# Copy simulation folder
	files="$files sim"

	# Copy poses
	files="$files poses"

	# Copy set-hostname script
	files="$files $MUROX_PROJECT/tools/robotTools/set-hostname.sh"

	# Copy the website
	#files="$files ${MUROX_PROJECT}/process/misc/rsb_ws_bridge_amiro/www"

	dest="root@${IP}:~/${NAME}/"
	sshpass -p$PASS rsync -v --progress -ua $files $dest

	# Copy the maps
	sshpass -p$PASS rsync -v --progress -au ${MUROX_PROJECT}/demo/CoreSLAMLocalization/data/* "$dest/data/"

done
