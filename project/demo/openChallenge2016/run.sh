#!/bin/sh

if [ -z "${1}" ]; then
  echo "Set an ID for the AMiRo"
  exit 1
fi

ID=${1}

echo "Final setup: ToBI <- AMiRo ID 1 <- AMiRo ID 0"
echo "Start AMiRo with ID ${ID}"

sleep 2

./stop.sh

cpufreq-set -g performance

spread -c amirospread &

sleep 5

# sensing camera from 'project/sense/senseCamJpg/'
# v4l2-ctl -d/dev/v4l-subdev8 --set-ctrl=vertical_flip=1
# ./senseCamJpg -d /dev/video6 -o /cam &

# sensing lidar from 'project/sense/senseHokuyo/'
./senseHokuyo -d /dev/ttyACM0 -o /lidar &

# following from 'project/process/followToBI/'
./follow_LaserScanner --lidarinscope /lidar &

# waypoint program from 'project/sandbox/waypoint/'
./waypoint --lidarinscope /lidar &

# start state machine
./final2016 --id ${ID} --turnAfterFollow 90 &
#--moveAfterFollow -300 &

./sendOdometryProtoPose --resetodom true -o /odom &

./CoreSLAM --odominscope /odom --lidarinscope /lidar --senImage 0 --delta 0.05 --sigma_xy 0.1 --sigma_xy_new_position 0.1 --sigma_theta 0.05 --sigma_theta_new_position 0.05  --remotePort 4823 --doMapUpdate false --loadMapWithValidPositionsFromPNG ./data/arenaEindhovenValid.png --loadMapFromImage ./data/arenaEindhoven.png --hominginscope /homing --flipHorizontal true &

./actTargetPosition --inscope /targetPositions

# webserver running on port 80
# ./rsb_ws_bridge_amiro &

wait
cpufreq-set -g ondemand
