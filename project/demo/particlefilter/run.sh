#!/bin/sh

./stop.sh

# RSB scopes
lidarscope='/AMiRo_Hokuyo/lidar'
odomscope='/AMiRo_Hokuyo/gps'

# start spread
spread &
sleep 5

# start laser sensing
./senseHokuyo --outscope "$lidarscope" &

# start odometry sensing
./sendOdometryProtoPose --outscope "$odomscope" &

# start particle filter
./particlefilter \
  --lidarInScope "$lidarscope" --odomInScope "$odomscope" \
  --pathToMap ./centralLab-clean-cropped-valid-4-scale-0.5.png --meterPerPixel 0.02 \
  --sampleCount 250

echo ""
echo "==== stopping ===="
echo ""

./stop.sh

