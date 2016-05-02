#!/bin/sh

killall -9 objectFetchTask
killall -9 senseFloorProximity
killall -9 senseHokuyo

sleep 1

./stopAMiRo


