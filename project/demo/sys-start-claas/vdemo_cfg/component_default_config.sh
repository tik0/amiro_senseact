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

export VDEMO_component_spread_upper="spread,$upper, -c  -g net -n -L 0 -w 2:"
export VDEMO_component_spread_lower="spread,$lower, -c  -g net -L 0 -w 2:"
export VDEMO_component_spread_rsb="rsb_spread,$spreadhost, -c  -g net -L 0 -w 2:"
export VDEMO_component_xcf_core="xcf_core,$upper, -c -s DispatcherControl -g net -n -L 1 -w 7: "

export VDEMO_component_texttospeech="text_to_speech,$upper, -c -g speech -w 2 -L 3: "
export VDEMO_component_speech_rec="speech_rec,$upper, -c -g speech -p psa -L 3 -w 2 -x: "

export VDEMO_component_door_detector="door_detector,$lower, -c  -x -l -d -1 -g rec -L 8: "
export VDEMO_component_object_recognition="object_recognition,$lower, -l -c  -g rec -L 8: "
export VDEMO_component_object_detection="object_detection,$lower, -c  -g rec -L 8: "
export VDEMO_component_waving_detection="waving_detection,$upper, -c  -g rec -L 8 -d -1: "
export VDEMO_component_face_recognition="facerec,$upper, -c  -g rec -L 8 -l -d -1: "

export VDEMO_component_navigation="navigation,$lower, -c  -g nav -L 1 -w 12 -d -1: "
export VDEMO_component_nav4xcf="xcf_navigation,$upper, -c  -g nav -L 4 -w 3: "
export VDEMO_component_nav4rsb="rsb_navigation,$upper, -c  -l -g nav -L 4 -w 3: "
export VDEMO_component_nav_gui="gui_navigation,$upper, -c  -g nav -L 4 -x: "

export VDEMO_component_memory_shortterm="shortterm_memory,$upper, -c  -s ShortTerm -g db -L 2 -w 2: "
export VDEMO_component_memory_scene="scene_memory,$upper, -c  -s Scene -g db -L 2 -w 2: "
export VDEMO_component_rsbam_shortterm="shortterm_rsbam,$upper, -c  -s ShortTerm -g db -L 2 -w 2: "
export VDEMO_component_rsbam_scene="scene_rsbam,$basepc, -s Scene -g db -L 2 -w 2: "
export VDEMO_component_memory_gui="gui_memory,$upper, -c  -n -g db -L 2 -x: "

export VDEMO_component_legdetector="legdetector,$lower, -c  -l -L 5 -g pt -w 2 -x: "
export VDEMO_component_bodydetector="bodydetector,$lower, -c  -L 5 -g pt -w 2 -x: "
export VDEMO_component_objectbuilder="objectbuilder,$lower, -c  -l -L 5 -g pt -w 2 -x: "
export VDEMO_component_objectbuilder_gui="gui_objectbuilder,$lower, -c  -n -L 5 -g pt -w 2: "

export VDEMO_component_arm_control="arm_control,$lower, -c  -s armControlServer -g hw -L 7 -n: "
export VDEMO_component_arm_action_graspup="arm_action_graspup,$upper, -c  -g hw -L 7 -n: "
export VDEMO_component_arm_action_home="arm_action_home,$upper, -c  -g hw -L 7 -n: "
export VDEMO_component_arm_action_side_graspup="arm_action_side_graspup,$upper, -c  -g hw -L 7 -n: "
export VDEMO_component_arm_action_foldup="arm_action_foldup,$upper, -c  -g hw -L 7 -n: "
export VDEMO_component_arm_GUI="arm_GUI,$upper, -c  -g hw -L 7 -n: "

export VDEMO_component_statemachine_gui="statemachine,$upper, -c  -n -x -d -1 -l -g tasks -L 9: "
export VDEMO_component_statemachine_gui_lower="statemachine,$lower, -c  -n -x -d -1 -l -g tasks -L 9: "

export VDEMO_component_kinect_floor_calibration="kinect_floor_calibration,$upper, -c  -x -n -l -d -1 -g kfc -L 8: "
export VDEMO_component_asm="asm,$lower, -c  -x -l -g asm -L 8: "

export VDEMO_component_rsb_host_monitor_upper="rsb_host_monitor,$upper, -c  -l -g logging -L 9: "
export VDEMO_component_rsb_host_monitor_lower="rsb_host_monitor,$lower, -c  -l -g logging -L 9: "
export VDEMO_component_logging_controller="logging_controller,$upper, -c  -l -g logging -L 9: "

export VDEMO_component_rsb_logger="rsb_logger,$upper, -c  -n -d -1 -l -g logging -L 9: "
export VDEMO_component_rsb_monitor="rsb_monitor,$upper, -c  -n -d -1 -l -g logging -L 9: "

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

export VDEMO_component_group_util="\
$VDEMO_component_rsb_logger\
$VDEMO_component_rsb_monitor\
"

export VDEMO_component_group_speech="\
$VDEMO_component_texttospeech\
$VDEMO_component_speech_rec\
"
export VDEMO_component_group_memory="\
$VDEMO_component_rsbam_scene\
"

export VDEMO_component_group_net="\
$VDEMO_component_spread_lower\
"

export VDEMO_component_group_nav="\
$VDEMO_component_navigation\
$VDEMO_component_nav4rsb\
$VDEMO_component_nav_gui\
"

export VDEMO_component_group_arm="\
$VDEMO_component_arm_control\
$VDEMO_component_arm_action_graspup\
$VDEMO_component_arm_action_home\
$VDEMO_component_arm_action_side_graspup\
$VDEMO_component_arm_action_foldup\
$VDEMO_component_arm_GUI\
"

export VDEMO_component_group_pt="\
$VDEMO_component_legdetector\
$VDEMO_component_legdetector_rsb\
$VDEMO_component_bodydetector\
$VDEMO_component_objectbuilder\
$VDEMO_component_objectbuilder_rsb\
$VDEMO_component_objectbuilder_gui\
"

export VDEMO_component_group_default="\
$VDEMO_component_group_net\
$VDEMO_component_group_memory\
$VDEMO_component_group_nav\
$VDEMO_component_group_speech\
$VDEMO_component_statemachine_gui\
$VDEMO_component_statemachine_gui_lower\
$VDEMO_component_group_logging\
$VDEMO_component_group_util\
"

