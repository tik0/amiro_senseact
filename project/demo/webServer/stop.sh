#!/bin/sh

# kill all sensing programs
killall -9 senseHokuyo
killall -9 senseCamJpg

# kill the webserver
killall -9 rsb_ws_bridge_amiro

# kill all spreads
killall -9 spread

cpufreq-set -g ondemand
