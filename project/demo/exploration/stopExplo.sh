#!/bin/sh

killall -9 senseFloorProximity 
killall -9 mapGenerator 
killall -9 motorControl
killall -9 senseRingProximity 
killall -9 frontierExploration 
killall -9 localPlannerISY
./stopAMiRo
