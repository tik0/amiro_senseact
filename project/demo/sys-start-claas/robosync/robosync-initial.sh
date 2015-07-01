#!/bin/bash

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
echo " Doing sanity checks . . . "
if [ ${myuser} != "biron" ]; then
    echo "You are not user 'biron'. Do you still want to continue? [y/n]"
    read answer
    if [ ${answer} != "y" -a ${answer} != "Y" ]; then 
	echo "exiting"
	exit 1
    fi
fi

group=`ssh ${remoteuser}@${host} ls -ld $prefix | awk '{print $4 }'`
groupid=`ssh ${remoteuser}@${host} getent group | grep $group | cut -d: -f3`

grep $group /etc/group > /dev/null
if [ $? -ne 0 ]; then
    echo " Could not find group '$group'"
    echo " Adding group $group($groupid) and adding current user to that group . . . "
    cmdg="sudo groupadd -g $groupid $group || (echo 'groupadd failed' && exit)"
    cmdu="sudo usermod -a -G $group ${myuser} || (echo 'usermod failed' && exit)"
    if [ $dry -eq 1 ]; then echo $cmdg; else eval $cmdg; fi
    if [ $dry -eq 1 ]; then echo $cmdu; else eval $cmdu; fi
fi

echo " Checking directories and adding them if needed . . . "
cmd0="sudo mkdir -p /vol"
cmd1="sudo chown -R ${myuser}:$group /vol || (echo 'chown failed' && exit)"
cmd2="mkdir -p /vol/ai /vol/ai/{share,lib} /vol/clf $prefix"
if [ $dry -eq 1 ]; then echo $cmd0; else eval $cmd0; fi
if [ $dry -eq 1 ]; then echo $cmd1; else eval $cmd1; fi
if [ $dry -eq 1 ]; then echo $cmd2; else eval $cmd2; fi

set -e

echo " Syncing $prefix . . . "
cmd="rsync -a --delete -e 'ssh' ${remoteuser}@${host}:$prefix/ $prefix"
if [ $dry -eq 1 ]; then echo $cmd; else eval $cmd; fi
echo " Syncing ai-modulefiles . . . "
cmd="rsync -a --delete -e 'ssh' --exclude='sbcl' ${remoteuser}@${host}:/vol/ai/share/modulefiles /vol/ai/share/"
if [ $dry -eq 1 ]; then echo $cmd; else eval $cmd; fi
echo " Syncing clf-modulefiles .. "
cmd="rsync -a --delete -e 'ssh' ${remoteuser}@${host}:/vol/clf/share/modulefiles /vol/clf/share/ "
if [ $dry -eq 1 ]; then echo $cmd; else eval $cmd; fi
echo " Syncing ai-tcl-modules .. "
cmd="rsync -a --delete -e 'ssh' ${remoteuser}@${host}:/vol/ai/lib/modules-tcl /vol/ai/lib/"
if [ $dry -eq 1 ]; then echo $cmd; else eval $cmd; fi

grep module ${HOME}/.bashrc > /dev/null
if [ $? -ne 0 ]; then 
    echo " Adding module alias to bashrc . . . "
    bashtemp=`tempfile`
    echo '
# Check for the init scripts existence for the rare case
# you login to another group where /vol/ai is not mounted
# or the init script has vanished somehow.
if [ -f /vol/ai/lib/modules-tcl/init/bash ]; then
    . /vol/ai/lib/modules-tcl/init/bash
    # You can use .modulefiles in your home for private modules
    # e.g. testing purposes or nobody else would need them.
    module use /vol/ai/share/modulefiles
    module use /vol/clf/share/modulefiles

    # If you start a nested shell (e.g. by typing bash) you might
    # not like modules you unloaded beeing reloaded in the nested shell
    # again if you load them in your .bashrc.
    # If reloading is the intended behavior remove the surrounding if
    if [ -z "$BASHRC_LOADED" ]; then
        # Here you can specifiy modules you always want to load
        export BASHRC_LOADED=1
    fi
fi
' > ${bashtemp}
    mergefile=`tempfile`
    if [ $dry -eq 0 ]; then cat ~/.bashrc ${bashtemp} > ${mergefile}; fi
    if [ $dry -eq 0 ]; then cp ${mergefile} ~/.bashrc; fi
    if [ $dry -eq 0 ]; then rm ${mergefile} ${bashtemp}; fi
fi
echo '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
echo '~     Syncing complete       ~'
echo '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
