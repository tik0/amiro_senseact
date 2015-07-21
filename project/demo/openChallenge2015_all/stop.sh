#!/bin/sh

# kill all commanding statemachines
killall -9 answerer_tobi

# kill all listening statemachines
killall -9 openChallengeGEPTRO_2nd
killall -9 answerer

# kill all secondary moving programs
killall -9 drivingObjectDetection
killall -9 objectDelivery

# kill all primary moving programs
killall -9 exploDriveEdge
killall -9 localPlanner

# kill localization
killall -9 CoreSLAM

# kill all sensing programs
killall -9 rirReader
killall -9 senseOdometry
killall -9 senseHokuyo

sleep 1

# stop all outputs
./stopAMiRo


