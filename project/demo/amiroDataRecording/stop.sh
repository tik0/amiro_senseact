#!/bin/sh
killall -9 senseRingProximity
killall -9 senseFloorProximity
killall -9 senseCamJpg
killall -9 sendOdometryProtoPose
killall -9 senseHokuyo
killall -9 spread
cpufreq-set -g ondemand