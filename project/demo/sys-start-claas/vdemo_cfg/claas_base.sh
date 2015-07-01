DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "$DIR/vdemo_aux_functions.sh"
source "$DIR/claas_defines.sh"

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

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${vdemo_prefix}/lib

##############################################################################
# DEFINE WHERE THE LOGGING DATA GO
##############################################################################
export VDEMO_logPath=/tmp/log/${USER}
export VDEMO_watchfile=/tmp/log/VDEMO_CLAAS_${USER}_${$}.log
export VDEMO_logfile_prefix=${VDEMO_logPath}/VDEMO_component_
mkdir -m 2777 -p ${VDEMO_logPath}

##############################################################################
# SETUP PC NAMES
##############################################################################

export basepc=`hostname -s`
export SPREADCONFIG=${VDEMO_demoRoot}/data/spread/$basepc
export SPREADCONFIG_RSB=${VDEMO_demoRoot}/data/spread/${basepc}-rsb
export car=$basepc
export cam=$basepc
export upper=$basepc
export lower=$basepc
export VDEMO_machine_setup="$VDEMO_demoRoot/robot_setup/$basepc"
export VDEMO_component_default_config="$VDEMO_demoRoot/component_default_claas_config.sh"

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

export VDEMO_simulation=true
if [ -e "$VDEMO_machine_setup" ]
then
  source "$VDEMO_machine_setup"
  export VDEMO_alert_string="Running mobile with conf: $VDEMO_machine_setup"
  export VDEMO_robot_config_loaded="TRUE";
  export VDEMO_simulation=false
fi

export spreadhost=$cam

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
