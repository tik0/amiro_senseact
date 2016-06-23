#!/bin/sh

set -e

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

# waypoint program from 'project/sandbox/waypoint/'
./waypoint --lidarinscope /lidar &

# start state machine
./final2016 --id ${ID} &

./sendOdometryProtoPose --resetodom true -o /odom &

# positions:
# door to amilab (30)
# first table (36)
# kitchen (37)
# next to VR lab (where the shelf used to be) (39)
# 
# 36 39
# 30 37
targetPoses="3613.16 1791.69 86.8487
5473.75 8436.29 178.356
3413.83 13116.3 -1.27339
3276.9 6502.31 -3.15126"

targetPose="$(echo "$targetPoses" | head -n $((${ID} + 1 % 4)) | tail -n 1)"

initialPose="$(cat ./poses/startPose.txt)"
initialX="$(echo "$initialPose" | cut -d\  -f 1)"
initialY="$(echo "$initialPose" | cut -d\  -f 2)"
initialTheta="$(echo "$initialPose" | cut -d\  -f 3)"

./CoreSLAM --odominscope /odom --lidarinscope /lidar --hominginscope /homing --mapAsImageOutScope /CoreSLAMLocalization/image --setPositionScope /setPosition/${ID} --targetPoseInScope /setTargetPose/${ID} --positionOutScope /amiro${ID}/pose --pathOutScope /amiro${ID}/path \
  --remotePort 4823 \
  --senImage 0 \
  --delta 0.05 --sigma_xy 10 --sigma_xy_new_position 100 --sigma_theta 0.1 --sigma_theta_new_position 0.15  --doMapUpdate false \
  --loadMapFromImage ./data/centralLab.png --flipHorizontal true \
  --erosionRadius 0.33 \
  --initialX $initialX --initialY $initialY --initialTheta $initialTheta \
  --targetPose "$targetPose" \
  --precomputeOccupancyMap true &

./actEmergencyStop --lidarinscope /lidar --cntMax 25 --distance 0.30 --delay 10 --switchinscope /following --doEmergencyBehaviour > /dev/null &

./actTargetPosition --inscope /targetPositions &

./actAmiroMotor &

wait
cpufreq-set -g ondemand
