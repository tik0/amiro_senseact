#!/bin/bash

trap './stop.sh; exit' EXIT

./stop.sh

# RSB scopes
lidarscope='/AMiRo_Hokuyo/lidar'
odomscope='/AMiRo_Hokuyo/odom'
kinscope='/AMiRo_Hokuyo/diffKin'

# start spread
spread &
sleep 5

# start gazebo
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

