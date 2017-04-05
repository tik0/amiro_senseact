#!/bin/bash
[[ $_ != $0 ]] && SOURCED=1 || SOURCED=0;
THIS_PWD=${PWD}
if [[ $SOURCED == 0 ]]; then
  echo "Run 'source $0'"
else
  test -e build && rm -r build
  test -e devel && rm -r devel
  source /opt/ros/$(ls /opt/ros/ | tail -n1)/setup.bash
  cd ${THIS_PWD}/src
  catkin_init_workspace
  cd ${THIS_PWD}
  export ROSCONSOLE_CONFIG_FILE=${THIS_PWD}/rosconsole.config
  catkin_make
  source ${THIS_PWD}/devel/setup.bash
fi
