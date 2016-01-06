#!/bin/sh

# kill all commanding statemachines
killall -9 commandSimulation

# kill all listening statemachines
killall -9 stateMachineGEPTRO
killall -9 amiroEnvironment

# kill all secondary moving programs
killall -9 drivingObjectDetection
killall -9 objectDelivery

# kill all primary moving programs
killall -9 exploDriveEdge
killall -9 localPlanner

# kill localization
killall -9 CoreSLAM

# kill all sensing programs
killall -9 senseRingProximity
killall -9 sendOdometryProtoPose
killall -9 senseHokuyo

sleep 1

# stop all outputs
./stopAMiRo


