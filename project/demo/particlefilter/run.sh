#!/bin/sh

./stop.sh

trap './stop.sh; exit;' INT TERM

# RSB scopes
lidarscope='/AMiRo_Hokuyo/lidar'
odomscope='/AMiRo_Hokuyo/gps'

# start spread for RSB communication
spread &
sleep 5

# start laser sensing
./senseHokuyo --outscope "$lidarscope" &

# start odometry sensing
./sendOdometryProtoPose --outscope "$odomscope" &

# start particle filter
./particlefilter \
  --lidarInScope "$lidarscope" --odomInScope "$odomscope" \
  --pathToMap ./twb-empty-0.01m_per_pixel.png --meterPerPixel 0.01 \
  --sampleCount 1000 \
  --newSampleProb 0 \
  --beamskip 2 \
  --maxFrequency 1
#  --debugImageOutScope /image

./stop.sh
