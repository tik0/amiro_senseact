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
    host=neodymium.techfak.uni-bielefeld.de
    remoteuser=${myuser}
else
    echo $arg | grep "@" > /dev/null
    if [ $? -eq 0 ]; then
	remoteuser=`echo $arg | cut --delimiter='@' --fields=1`
	host=`echo $arg | cut --delimiter='@' --fields=2`
    else
	remoteuser=${myuser}
	host=$arg
    fi
fi

echo " Syncing with address ${host} . . . "
echo " Syncing /vol/robocup . . . "

cmd="rsync -al --delete -e 'ssh' --exclude='$prefix/etc' ${remoteuser}@${host}:$prefix/ $prefix"
cmd_etc="rsync -al -e 'ssh' ${remoteuser}@${host}:$prefix/etc/ $prefix/etc"

echo "  $cmd"
echo "  $cmd_etc"

if [ $dry -eq 0 ]; then
	eval $cmd
	eval $cmd_etc
fi

echo '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
echo '~     Syncing complete       ~'
echo '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
