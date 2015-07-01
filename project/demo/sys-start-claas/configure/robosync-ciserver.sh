#!/bin/bash
set -e

trap ctrl_c INT
function ctrl_c() {
        echo "** Trapped CTRL-C"
        exit 1
}

if [ ! -n "$prefix" ]; then
	echo "Environment variable \"\$prefix\" must be set!"
	exit 1
fi

# remove training slash from prefix path
prefix=${prefix%/}

# parse options
dry=0
while [[ $# > 0 ]]
do
	key="$1"
	shift

	case $key in
	-d|--dry)
		dry=1
		#shift
		;;
	*)
		# unknown option
		arg=$key
		;;
	esac
done

myuser=`whoami`


if [[ -z $arg ]]; then
    host=nohost
    remoteuser=${myuser}
else
    echo $arg | grep "@" > /dev/null
    if [ $? -eq 0 ]; then
	remoteuser=`echo $arg | cut --delimiter='@' --fields=1`
	host=`echo $arg | cut --delimiter='@' --fields=2`
    else
	remoteuser=biron
	host=multivac.dhcp
    fi
fi

echo " Syncing to address ${host} . . . "
echo " Syncing /vol/robocup . . . "

cmd="rsync -v -al --delete -e 'ssh' --exclude='$prefix/etc' $prefix/ ${remoteuser}@${host}:$prefix"
cmd_etc="rsync -v -al -e 'ssh' $prefix/etc/ ${remoteuser}@${host}:$prefix/etc"

echo "  $cmd"
echo "  $cmd_etc"

if [ $dry -eq 0 ]; then
	eval $cmd
	eval $cmd_etc
fi

echo '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
echo '~     Syncing complete       ~'
echo '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
