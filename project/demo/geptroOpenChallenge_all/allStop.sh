#!/bin/sh

./stop.sh

sleep 1

# kill all loading programs
killall -9 objectDetection

# kill spread
killall -9 spread
