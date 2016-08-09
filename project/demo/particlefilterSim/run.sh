#!/bin/bash

trap 'echo got SIGINT or script is exiting; ./stop.sh; exit' EXIT SIGINT

./stop.sh

# RSB scopes
lidarscope='/AMiRo_Hokuyo/lidar'
odomscope='/AMiRo_Hokuyo/odom'
kinscope='/AMiRo_Hokuyo/diffKin'
imagescope='/image'
poseestimatescope='/setPosition'

# start spread if not running
if ps -C spread && test -S /tmp/4803; then
	echo "spread is still running"
else
	echo "spread is not running, starting it"
	spread &
	sleep 5
fi

# show map and set pose estimate
./setPosition/setPosition --i "$imagescope" --o "$poseestimatescope" &

# start gazebo (simulation of AMiRo etc.)
gazebo ./data/T.world &

# start particle filter
#valgrind --tool=callgrind \
./particlefilter/particlefilter \
  --lidarInScope "$lidarscope" --odomInScope "$odomscope" \
  --pathToMap ./data/T-scale-0.5.png --meterPerPixel 0.02 \
  --sampleCount 500 \
  --newSampleProb 0.1 \
  --beamskip 1 \
  --debugImageOutScope /image &

# start keyboard control
./sendTwistControls/sendTwistControls -o "$kinscope"

echo ""
echo "==== stopping ===="
echo ""

./stop.sh

