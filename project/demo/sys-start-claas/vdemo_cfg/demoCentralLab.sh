
# Find out the local directory this script lives in:
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "$DIR/robocup_base.sh"

##############################################################################
##############################################################################
#####                    Demo Fbiis Lab                                  #####
##############################################################################
##############################################################################

##############################################################################
# TASK SPECIFIC CONFIGURATION VARIABLES
##############################################################################
# robot dependent configurations for components

#Bonsai Task
export PATH_TO_BONSAI_CONFIG="${PATH_TO_BONSAI_CONFIG}/demoCentralLabConfig.xml"
export PATH_TO_TASK_SCXML="${PATH_TO_TASK_SCXML}/demoCentralLab.xml"

#pocketsphinx_grammars
export VDEMO_PSA_CONFIG="${PATH_TO_PSA_CONFIG}/demo/demoCentralLab.conf"

#Map
export MAP="$DEFAULT_MAP"
#export MAP="centralLab"

# which communication to use for person tracking pipeline
export PT_PIPELINE="rsb"

export DEBUG_MODE="TRUE"

##############################################################################
# COMPONENT LIST
##############################################################################
# set components to start on which host, format as follows:
# component specifiers seperated by ': '
# each component specifier has three fields seperated by ',':
#    1. a host name where the component should run
#    2. components name: name of the component, a respective component
#       script for must be available in $VDEMO_scripts, with the
#       following name: component_<component>.sh
#    3. optional argument for automatic start (wait time, wait for XCF
#       registration...) 
#       known option include:
#        -w <n> wait for process to start completly (numeric: seconds)
#	     -s <s> wait for XCF server (string: name of XCF server)
#    	 -p <s> wait for XCF publisher (string: name of XCF publisher)
#        -l     activate initial logging for the component (must be activated
#               before the component is started!)
#        -x     start own X server for the component
#        -n     do not include in autostart
#        -g <s> allow to define a group (string: name of group)
#        -L <n> component level, affects starting order (numeric: level)
#        -d <n> detach time, automatically detaches screen after n seconds, or
#               leaves it open all the time (-1), default is 10 seconds
# BE CAREFUL: No character after line end ('\') because the shell is very picky
# about that!!!
export VDEMO_components="\
$VDEMO_component_group_default \
$VDEMO_component_group_pt \
$VDEMO_component_waving_detection \
$VDEMO_component_object_recognition \
$VDEMO_component_asm \
$VDEMO_component_group_arm \
surfacedetector,$lower, -n -L 3 -g pt -w 3 -x: \
attention,$lower, -n -L 4 -w 2 -l: \
"
