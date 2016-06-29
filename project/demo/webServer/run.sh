#!/bin/sh

./stop.sh

cpufreq-set -g performance

spread -c amirospread &

sleep 5

# sensing camera from 'project/sense/senseCamJpg/'
v4l2-ctl -d/dev/v4l-subdev8 --set-ctrl=vertical_flip=1
./senseCamJpg -d 6 -o /cam &

# sensing rgbd camera
./senseRGBD --rgbMode 0 --depthMode 0 -si -r /rgbd &

# sensing lidar from 'project/sense/senseHokuyo/'
./senseHokuyo -d /dev/ttyACM0 -o /lidar &

# sensing sick
./senseSick -o /lidar > /dev/null &

# webserver running on port 80
./rsb_ws_bridge_amiro &

wait
cpufreq-set -g ondemand
