#!/bin/bash

./stop.sh

# RSB scopes
lidarscope='/AMiRo_Hokuyo/lidar'
odomscope='/AMiRo_Hokuyo/gps'
kinscope='/AMiRo_Hokuyo/diffKin'

# start spread
spread &
sleep 5

# start gazebo
gazebo ./data/enclosed_world-no-cylinder.world &

# start particle filter
./particlefilter/particlefilter \
  --lidarInScope "$lidarscope" --odomInScope "$odomscope" \
  --pathToMap ./data/enclosed_world-clean2-cropped.png --meterPerPixel 0.01 \
  --sampleCount 100 &

# start keyboard control
./sendTwistControls/sendTwistControls -o "$kinscope"
