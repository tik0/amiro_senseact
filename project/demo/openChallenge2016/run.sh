#!/bin/sh

set -e

./set-hostname.sh

if [ -z "${1}" ]; then
  echo "Set an ID for the AMiRo"
  exit 1
fi

ID=${1}

echo "Start AMiRo with ID ${ID}"

sleep 2

trap './stop.sh; exit' INT TERM HUP PIPE

./stop.sh

wait_for_port()
{
	while test -n "$(netstat -an | grep $1 | grep -e LISTEN -e FIN_WAIT2 -e TIME_WAIT)"; do
		echo "port is still in use, waiting...";
		sleep 1;
	done
}

cpufreq-set -g performance

# local spread
wait_for_port 4803
spread &

# external spread
wait_for_port 4823
spread -c amirospread &
sleep 7

# sensing lidar from 'project/sense/senseHokuyo/'
if [ -e /dev/ttyACM0 ]; then
	echo "Assuming Hokuyo laser scanner"
	./senseHokuyo -d /dev/ttyACM0 -o /lidar &
else
	echo "Assuming SICK laser scanner"
	./senseSickTim -o /lidar > /dev/null &
fi

# sense ringproximty
./senseRingProximity --outscopeObstacle /rir_prox/obstacle --noEdgeValues &

# waypoint program from 'project/sandbox/waypoint/'
./waypoint --lidarinscope /lidar &

# start state machine
./final2016 --id ${ID} &

./sendOdometryProtoPose --resetodom true -o /odom &

# 0 1
# 2 3
initialPoses="33749.9 17848.6 -90
33250.2 17848.6 -90
33743.3 18348.3 -90
33250.2 18348.3 -90"

initialPose="$(echo "$initialPoses" | head -n $((${ID} + 1 % 4)) | tail -n 1)"
initialX="$(echo "$initialPose" | cut -d\  -f 1)"
initialY="$(echo "$initialPose" | cut -d\  -f 2)"
initialTheta="$(echo "$initialPose" | cut -d\  -f 3)"

# positions:
# 0: living room
# 1: dining_room
# 2: kitchen
# 3: corridor
kitchen="27400.7 18478 -90"
dining_room="29312.9 18658.4 -90"
#living_room="32758.5 18387.8 -90"
living_room="33250.2 18348.3 -90"
#corridor="27978 11695.1 90"
corridor="27830.3 14194.7 -90"
targetPoses="$dining_room
$kitchen
$corridor
$living_room"

# take row ID + 1
targetPose="$(echo "$targetPoses" | head -n $((${ID} + 1 % 4)) | tail -n 1)"

./CoreSLAM --odominscope /odom --lidarinscope /lidar --hominginscope /homing --mapAsImageOutScope /CoreSLAMLocalization/image --setPositionScope /setPosition/${ID} --targetPoseInScope /setTargetPose/${ID} --positionOutScope /amiro${ID}/pose --pathOutScope /amiro${ID}/path \
  --remotePort 4823 \
  --senImage 0 \
  --delta 0.05 --sigma_xy 10 --sigma_xy_new_position 100 --sigma_theta 0.1 --sigma_theta_new_position 0.15  --doMapUpdate false \
  --loadMapFromImage ./data/Leipzig_Arena_A.pgm --flipHorizontal true \
  --erosionRadius 0.33 \
  --initialX $initialX --initialY $initialY --initialTheta $initialTheta \
  --targetPose "$targetPose" \
  --precomputeOccupancyMap true &

./actEmergencyStop --lidarinscope /lidar --cntMax 25 --distance 0.30 --delay 10 --switchinscope /following --doEmergencyBehaviour > /dev/null &

./actTargetPosition --inscope /targetPositions &

./actAmiroMotor &

wait
cpufreq-set -g ondemand
