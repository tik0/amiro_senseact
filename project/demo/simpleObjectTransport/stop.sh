#!/bin/sh

killall -9 objectTransport
killall -9 senseRingProximity
killall -9 setLights

killall -9 spread

sleep 1

./stopAMiRo


