#!/bin/sh

# kill statemachine
killall -9 final2015

# kill all acting programs
killall -9 followToBI
killall -9 waypoint

# kill all sensing programs
killall -9 senseHokuyo

# kill all spreads
killall -9 spread

sleep 1

# reset AMiRo from 'project/act/stopAMiRo/'
./stopAMiRo
