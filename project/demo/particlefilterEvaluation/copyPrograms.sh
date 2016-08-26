#!/bin/sh

set -e

IP="${1:-10.0.0.21}"

if [ -z "$2" ]; then
	SSHPASS=""
else
	SSHPASS="sshpass -p$2 "
fi

echo "====================="
echo "copying to ${IP}"
echo "====================="

NAME=`basename "$PWD"`
$SSHPASS ssh root@${IP} "mkdir -p ~/${NAME}/"

# collect all executables
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
files="$files rsb.conf"

# Copy set-hostname script
#files="$files $MUROX_PROJECT/tools/robotTools/set-hostname.sh"

# The map
files="$files data/png/*.png"

dest="root@${IP}:~/${NAME}/"
$SSHPASS rsync -v --progress -ua $files $dest

# Copy the maps
#sshpass -p$PASS rsync -v --progress -au ${MUROX_PROJECT}/demo/CoreSLAMLocalization/data/* "$dest/data/"
#$SSHPASS rsync -v --progress -au ${MUROX_PROJECT}/demo/CoreSLAMLocalization/data/Leipzig_Arena_A.pgm "$dest/data/"
