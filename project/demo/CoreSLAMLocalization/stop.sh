#!/bin/sh
killall -9 CoreSLAM
killall -9 actAmiroMotor
killall -9 senseHokuyo
killall -9 sendOdometryProtoPose
killall -9 spread
killall -9 actTargetPosition

cpufreq-set -g ondemand

./stopAMiRo
