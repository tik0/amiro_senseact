#!/bin/sh

killall -9 objectFetchTask
killall -9 senseFloorProximity
killall -9 senseHokuyo
killall -9 senseSimple3DTo2D

sleep 1

./stopAMiRo


