#!/bin/bash
killall senseHokuyo
killall followToBI
killall waypoint
killall spread

# stopping motors from 'project/act/stopMotor/'
./stopMotor &
