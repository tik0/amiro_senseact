#!/bin/sh

if [ -z "${1}" ]; then
  echo "Set an ID for the AMiRo"
  exit 1
fi

ID=${1}

echo "Final setup: ToBI <- AMiRo ID 1 <- AMiRo ID 0"
echo "Start AMiRo with ID ${ID}"

sleep 2

trap './stop.sh; exit' INT TERM

./stop.sh

cpufreq-set -g performance

# local spread
while test -n "$(netstat -an | grep 4803 | grep LISTEN)"; do
	echo "port is still in use, waiting...";
	sleep 1;
done
spread &
# external spread
while test -n "$(netstat -an | grep 4823 | grep LISTEN)"; do
	echo "port is still in use, waiting...";
	sleep 1;
done
spread -c amirospread &

sleep 5

# sensing lidar from 'project/sense/senseHokuyo/'
./senseHokuyo -d /dev/ttyACM0 -o /lidar &

# waypoint program from 'project/sandbox/waypoint/'
./waypoint --lidarinscope /lidar &

# start state machine
./final2016 --id ${ID} &

./sendOdometryProtoPose --resetodom true -o /odom &

./CoreSLAM --odominscope /odom --lidarinscope /lidar --hominginscope /homing --mapAsImageOutScope /CoreSLAMLocalization/image --setPositionScope /setPosition/${ID} --positionOutScope /amiro${ID}/pose --pathOutScope /amiro${ID}/path \
  --remotePort 4823 \
  --senImage 0 \
  --delta 0.02 --sigma_xy 10 --sigma_xy_new_position 100 --sigma_theta 0.1 --sigma_theta_new_position 0.15  --doMapUpdate false \
  --loadMapWithValidPositionsFromPNG ./data/centralLab-clean-cropped-valid-4-scale-0.5.png --loadMapFromImage ./data/centralLab-clean-cropped-4-scale-0.5.png \
  --erosionRadius 0.3 \
  --initialX 4244.11 --initialY 6446.25 --initialTheta 6.71772 \
  --targetPose 6000 6000 90 \
  --precomputeOccupancyMap true &

./actEmergencyStop --lidarinscope /lidar --cntMax 25 --distance 0.15 --delay 10 --switchinscope /following --doEmergencyBehaviour > /dev/null &

./actTargetPosition --inscope /targetPositions &

wait
cpufreq-set -g ondemand
