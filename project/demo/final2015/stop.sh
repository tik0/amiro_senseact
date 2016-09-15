#!/bin/sh

# kill statemachine
killall -9 final2015

# kill all acting programs
killall -9 follow_LaserScanner
killall -9 waypoint

# kill all sensing programs
killall -9 senseHokuyo
killall -9 senseSickTim
killall -9 senseCamJpg

# kill the webserver
killall -9 rsb_ws_bridge_amiro

# kill all spreads
killall -9 spread

sleep 1

# reset AMiRo from 'project/act/stopAMiRo/'
./stopAMiRo
