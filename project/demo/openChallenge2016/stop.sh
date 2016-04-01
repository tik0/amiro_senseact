#!/bin/sh

# kill statemachine
killall -9 final2016

# kill all acting programs
killall -9 follow_LaserScanner
killall -9 waypoint

# kill all sensing programs
killall -9 senseHokuyo
killall -9 senseCamJpg
killall -9 sendOdometryProtoPose
killall -9 CoreSLAMLocalization

# kill the webserver
killall -9 rsb_ws_bridge_amiro

# kill all spreads
killall -9 spread

sleep 1

cpufreq-set -g ondemand

# reset AMiRo from 'project/act/stopAMiRo/'
./stopAMiRo
