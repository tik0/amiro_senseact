#!/bin/sh

# kill all commanding statemachines
killall -9 commandSimulation

# kill all listening statemachines
killall -9 stateMachineGEPTRO
killall -9 amiroEnvironment

# kill all third level moving programs
killall -9 drivingObjectDetection
killall -9 objectDelivery

# kill all secondary moving programs
killall -9 localPlannerISY

# kill all primary moving programs
killall -9 motorControl

# kill localization and exploration
killall -9 mapGenerator 
killall -9 frontierExploration 

# kill all sensing programs
killall -9 rirReader
killall -9 senseFloorProximity 

sleep 1

# stop all outputs
./stopAMiRo


