Sense Act Plan/ROS project
====

How To Setup Repo
==
* clone all submodules ```git submodule update --init --recursive```
* add environments to bashrc ```cd project/utilities && chmod +x initMurox && ./initMurox && cd ../../```
* create protobuff files ```cd project/includes/types && chhmod +x createPbFiles.sh && ./createPbFiles <rsb.version> && cd ../../../```
* copy rsb.conf to ROS-home ```cp catkin_ws/rsb.conf ~/.ros/```
* to build ROS workspace: ```cd catkin_ws && source devel/setup.bash && catkin_make``` if there is no devel folder then ```source /opt/ros/kinetic/setup.bash``` instead
