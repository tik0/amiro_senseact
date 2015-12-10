#!/bin/sh

./stop.sh

cpufreq-set -g performance

spread -c amirospread &

sleep 5

# sensing camera from 'project/sense/senseCamJpg/'
v4l2-ctl -d/dev/v4l-subdev8 --set-ctrl=vertical_flip=1
./senseCamJpg -d /dev/video6 -o /cam &

# sensing lidar from 'project/sense/senseHokuyo/'
./senseHokuyo -d /dev/ttyACM0 -o /lidar &

# webserver running on port 80
./rsb_ws_bridge_amiro &

wait
cpufreq-set -g ondemand
