#!/bin/sh

./stop.sh

# kill all sensing programs
killall -9 objectDetection

# kill spread
killall -9 spread

sleep 1

# stop all outputs
./stopAMiRo


