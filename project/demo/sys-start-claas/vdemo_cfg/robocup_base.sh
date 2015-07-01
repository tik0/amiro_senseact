DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "$DIR/vdemo_aux_functions.sh"
source "$DIR/robocup_defines.sh"

##############################################################################
# OBLIGATORY VDEMO SETUP VARIABLES (derived from Makefile or environment)
##############################################################################

# root of the demo
export VDEMO_demoRoot=${vdemo_demoroot}
# Path to find the component scripts:
export VDEMO_componentPath=${vdemo_demoroot}/component_scripts

##############################################################################
# ENVIRONMENT
##############################################################################

# set log permissions #
install -d /tmp/log/ -m 777

export prefix=$vdemo_prefix

# location of java
JAVA_HOME=/usr/lib/jvm/java-7-openjdk-amd64
JRE_HOME=$JAVA_HOME/jre
export JAVA_HOME JRE_HOME
export PATH="${JAVA_HOME}/bin:${vdemo_prefix}/bin:$PATH"

export LD_LIBRARY_PATH=${vdemo_prefix}/lib

export PLAYERCONFIG=${VDEMO_demoRoot}/data/playerconfig/p2os_BIRON_Desktop.cfg

# location of bonsai configs and state machines
export PATH_TO_BONSAI_CONFIG="${vdemo_prefix}/opt/robocup-statemachine/etc/bonsai_configs"
export PATH_TO_TASK_SCXML="${vdemo_prefix}/opt/robocup-statemachine/etc/state_machines"
export PATH_TO_PSA_CONFIG="${vdemo_prefix}/share/SpeechRec/psConfig"
export PATH_TO_OBJECT_MODELS="${VDEMO_demoRoot}/data/clafu_models"
export PATH_TO_ANNOTATIONS="${VDEMO_demoRoot}/data/annotations"
export OBJECT_MODELS="${PATH_TO_OBJECT_MODELS}/${DEFAULT_MODELS}"

##############################################################################
# DEFINE WHERE THE LOGGING DATA GO
##############################################################################
export VDEMO_logPath=/tmp/log/${USER}
export VDEMO_watchfile=/tmp/log/VDEMO_BIRON_${USER}_${$}.log
export VDEMO_logfile_prefix=${VDEMO_logPath}/VDEMO_component_
mkdir -m 2777 -p ${VDEMO_logPath}

##############################################################################
# SETUP PC NAMES
##############################################################################

export basepc=`hostname -s`
export SPREADCONFIG=${VDEMO_demoRoot}/data/spread/$basepc
export SPREADCONFIG_RSB=${VDEMO_demoRoot}/data/spread/${basepc}-rsb
export upper=$basepc
export lower=$basepc
export VDEMO_robot_setup="$VDEMO_demoRoot/robot_setup/$basepc"
export VDEMO_component_default_config="$VDEMO_demoRoot/component_default_config.sh"

# use the modules.xml description from the distribution
export OPEN_NI_INSTALL_PATH="$prefix"

# defaults (overwritten by robot setup)
export VDEMO_robot_config_loaded="FALSE"
export RSB_PLUGINS_CPP_LOAD=rsbspread
export RSB_TRANSPORT_SPREAD_PORT=4803
export RSB_TRANSPORT_SPREAD_HOST=$basepc
export RSB_TRANSPORT_SPREAD_ENABLED=1
export RSB_TRANSPORT_SOCKET_ENABLED=0
export RSB_TRANSPORT_INPROCESS_ENABLED=0
export has_dslr=false
export has_asus=false

export VDEMO_simulation=true
if [ -e "$VDEMO_robot_setup" ]
then
  source "$VDEMO_robot_setup"
  export VDEMO_alert_string="Running mobile with conf: $VDEMO_robot_setup"
  export VDEMO_robot_config_loaded="TRUE";
  export VDEMO_simulation=false
fi

export spreadhost=$lower

if [ -z "$has_dslr" ]
then
  export VDEMO_has_dslr=false
else
  export VDEMO_has_dslr=$has_dslr
fi

if [ -z "$has_asus" ]
then
  export VDEMO_has_asus=false
else
  export VDEMO_has_asus=$has_asus
fi

if [ $upper ]; then
	#workaround for lower
	if [ $lower == `hostname` ];
	  then export XCF_Initial_Host="$upper"; 
	fi
fi

#allow lower to use pulseaudio from upper.
#pactl load-module module-native-protocol-tcp auth-ip-acl=`gethostip -d ${lower}`

##############################################################################
# MIDDLEWARE CONFIG
##############################################################################
export PT_PIPELINE="rsb"

##############################################################################
# COMPONENT LIST
##############################################################################
# In this basescript NO components should be defined. Better use:
if [ -e "$VDEMO_component_default_config" ]
then
  source "$VDEMO_component_default_config"
fi 

##############################################################################
# CLOSURE
##############################################################################

