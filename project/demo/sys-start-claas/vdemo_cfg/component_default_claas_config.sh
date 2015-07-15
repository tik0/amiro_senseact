##############################################################################
# DEFAULT COMPONENT SETUP
##############################################################################
# set components to start on which host, format as follows:munch
# component specifiers seperated by ': '
# each component specifier has three fields seperated by ',':
#    1. a host name where the component should run
#    2. components name: name of the component, a respective component
#       script for must be available in $VDEMO_scripts, with the
#       following name: component_<component>.sh
#    3. optional argument for automatic start (wait time, wait for XCF
#       registration...) 
#       known option include:
#   Options:
#      -w <n> wait n seconds for process to start completely
#      -c     run check every second during wait and break if component is running
#      -W <n> set delay when an asynchronous check is performed in case -w is not specified
#      -l     activate initial logging for the component
#      -x     use own X server for the component
#      -n     do not include in level/group/auto start
#      -g <s> allow to define a group (string: name of group)
#      -L <n> component level, affects starting order (numeric: level)
#      -d <n> detach time, automatically detaches screen after n seconds, or
#             leaves it open all the time (-1), default is 10 seconds
#      -t     title of the component / provide unique names for multiple instances on the same host
#      -v     export variable varname=var to component script
#
# BE CAREFUL: No character after line end ('\') because the shell is very picky
# about that!!!

##############################################################################
# COMPONENT LIST
##############################################################################

export VDEMO_component_spread_car="spread,$car, -c  -g net -n -L 0 -w 2:"
export VDEMO_component_spread_cam="spread,$cam, -c  -g net -L 0 -w 2:"
export VDEMO_component_spread_rsb="rsb_spread,$spreadhost, -c  -g net -L 0 -w 2:"

export VDEMO_component_hokuyo="hokuyo,$cam, -c  -g sense -L 1 -w 2:"
export VDEMO_component_gigEvisionImperX="gigEvisionImperX,$cam, -c  -g sense -L 1 -w 2:"
export VDEMO_component_gigEvisionManta="gigEvisionManta,$cam, -c  -g sense -L 1 -w 2:"
export VDEMO_component_LeuzeODS9ODS96B="LeuzeODS9ODS96B,$car, -c  -g sense -L 1 -w 2:"
export VDEMO_component_PepperlFuchsOMD8000="PepperlFuchsOMD8000,$car, -c  -g sense -L 1 -w 2:"
export VDEMO_component_OpenNI2="OpenNI2,$cam, -c  -g sense -L 1 -w 2:"
export VDEMO_component_PeakCan="PeakCan,$car, -c  -g sense -L 1 -w 2:"
export VDEMO_component_LASE_2000D_226="LASE_2000D_226,$car, -c  -g sense -L 1 -w 2:"
export VDEMO_component_Sick_LD_MRS_1="Sick_LD_MRS_1,$car, -c  -g sense -L 1 -w 2:"
export VDEMO_component_Sick_LD_MRS_2="Sick_LD_MRS_2,$car, -c  -g sense -L 1 -w 2:"
export VDEMO_component_S10Terminal="S10Terminal,$car, -c  -g sense -L 1 -w 2:"
export VDEMO_component_MachineModel="MachineModel,$car, -c -g util -L 1 -w 2:"

export VDEMO_component_rsb_webserver="rsb_webserver,$car, -c  -g webserver -L 2 -w 2:"

export VDEMO_component_rsbag="rsbag,$cam, -c -d -1 -g logging -L 3:"
export VDEMO_component_rsb_logger="rsb_logger,$car, -c  -n -d -1 -g logging -L 3:"
export VDEMO_component_rsb_monitor_car="rsb_monitor,$car, -c  -n -d -1 -g logging -L 3:"
export VDEMO_component_rsb_monitor_cam="rsb_monitor,$cam, -c  -n -d -1 -g logging -L 3:"

##############################################################################
# GROUPS
##############################################################################

if $VDEMO_simulation
then
export VDEMO_component_group_logging=""
else
export VDEMO_component_group_logging="\
"
fi

export VDEMO_component_group_net="\
$VDEMO_component_spread_cam\
$VDEMO_component_spread_car\
"

export VDEMO_component_group_sense="\
$VDEMO_component_hokuyo\
$VDEMO_component_PepperlFuchsOMD8000\
$VDEMO_component_LeuzeODS9ODS96B\
$VDEMO_component_OpenNI2\
$VDEMO_component_PeakCan\
$VDEMO_component_LASE_2000D_226\
$VDEMO_component_Sick_LD_MRS_1\
$VDEMO_component_Sick_LD_MRS_2\
$VDEMO_component_Sick_LD_MRS\
$VDEMO_component_gigEvisionManta\
$VDEMO_component_S10Terminal\
"

export VDEMO_component_group_util="\
$VDEMO_component_rsbag\
$VDEMO_component_rsb_logger\
$VDEMO_component_rsb_monitor_car\
$VDEMO_component_rsb_monitor_cam\
$VDEMO_component_MachineModel\
"

export VDEMO_component_group_web="\
$VDEMO_component_rsb_webserver\
"

export VDEMO_component_group_default="\
$VDEMO_component_group_net\
$VDEMO_component_group_sense\
$VDEMO_component_group_web\
$VDEMO_component_group_util\
"

