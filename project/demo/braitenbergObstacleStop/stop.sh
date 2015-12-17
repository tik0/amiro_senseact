#!/bin/sh

killall -9 frontObjectDetection
killall -9 braitenbergObstacleStop
killall -9 senseRingProximity

sleep 1

./stopAMiRo


