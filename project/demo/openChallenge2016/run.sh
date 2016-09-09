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

start_spread()
{
	# try to start spread 10 times
	for i in $(seq 10); do
		spread $@ &
		PID=$!
		sleep 1

		# check if it is running
		if ps -o pid= | grep -q "^ *$PID$"; then
			echo "spread $@ started successfully (try #$i)"
			return 0
		else
			echo "spread $@ not running (try #$i)"
		fi
	done

	echo "could not start spread"
	exit 1
}

cpufreq-set -g performance

# local spread
wait_for_port 4803
start_spread

# external spread
wait_for_port 4823
start_spread -c amirospread

# sensing lidar from 'project/sense/senseHokuyo/'
if [ -e /dev/ttyACM0 ]; then
	echo "Assuming Hokuyo laser scanner"
	./senseHokuyo -d /dev/ttyACM0 -o /lidar > /dev/null &
else
	echo "Assuming SICK laser scanner"
	./senseSickTim -o /lidar > /dev/null &
fi

# sense ringproximty
#./senseRingProximity --outscopeObstacle /rir_prox/obstacle --noEdgeValues --period 100 &

# waypoint program from 'project/sandbox/waypoint/'
#./waypoint --lidarinscope /lidar --range 1.5 --removeLessThan 9 --splitConnected 0.06 &
# 818 - 409
sleep 1 # sleep so we don't get old laser values
./waypoint --lidarinscope /lidar --range 1.5 --scanStartIndex 390 --scanEndIndex 420 -s > /dev/null &

# start state machine
poseXThresholds="13.85
15.7"
poseXThreshold="$(echo "$poseXThresholds" | head -n $((${ID} + 1 % 4)) | tail -n 1)"
./final2016 --id ${ID} --poseScope /amiro${ID}/pose --poseXThreshold $poseXThreshold > /dev/null &

./sendOdometryProtoPose --resetodom true -o /odom > /dev/null &

# start follower
# default values:
# followMinDist: 300 mm
# followMinBackDist: 200 mm
# followDistSlowingDown: 200 mm
# forwardSpeed: 500 mm/s
# forwardMinSpeed: 150 mm/s
# turningSpeed: 100 mrad/s
# turnCorrectSpeed: 60 mrad/s
# maxRange: 2000 mm
# rotationTolarence: PI/36 ~= 5°
./follow_LaserScanner -l /lidar --forwardSpeed 800 --forwardMinSpeed 400 --followMinDist 400 --followDistSlowingDown 400 --followMinBackDist 200 --maxRange 1500 --turningSpeed 50 --maxAngle 180 &

# 0 1
# 2 3
initialPoses="33749.9 17848.6 -90
33250.2 17848.6 -90
33743.3 18348.3 -90
33250.2 18348.3 -90"

initialPoses="19205.7 14267.7 45
22195.6 14475.4 -90
31262.3 17543.2 180"

initialPoses="13443.8 11059.1 -46
14855.9 10300.7 39.8219"

initialPose="$(echo "$initialPoses" | head -n $((${ID} + 1 % 4)) | tail -n 1)"
initialX="$(echo "$initialPose" | cut -d\  -f 1)"
initialY="$(echo "$initialPose" | cut -d\  -f 2)"
initialTheta="$(echo "$initialPose" | cut -d\  -f 3)"

# positions:
# 0: corridor
# 1: dining_room
# 2: living_room
# 3: kitchen

corridor="27830.3 14000.7 -90"
kitchen="25500.8 15824.3 142.125"
dining_room="28366.7 14903.5 0"
living_room="31807.2 18491.4 -90"

targetPoses="$corridor
$dining_room
$living_room
$kitchen"

# take row ID + 1
targetPose="$(echo "$targetPoses" | head -n $((${ID} + 1 % 4)) | tail -n 1)"

./CoreSLAM --odominscope /odom --lidarinscope /lidar --hominginscope /homing --mapAsImageOutScope /CoreSLAMLocalization/image --setPositionScope /setPosition/${ID} --targetPoseInScope /setTargetPose/${ID} --positionOutScope /amiro${ID}/pose --pathOutScope /amiro${ID}/path \
  --remotePort 4823 \
  --senImage 0 \
  --delta 0.05 --sigma_xy 10 --sigma_xy_new_position 100 --sigma_theta 0.1 --sigma_theta_new_position 0.15  --doMapUpdate false \
  --loadMapFromImage ./data/clf_final.pgm --flipHorizontal true \
  --erosionRadius 0.35 \
  --initialX $initialX --initialY $initialY --initialTheta $initialTheta \
  --targetPose "$targetPose" \
  --precomputeOccupancyMap true \
  --targetSpeed0 0 --targetSpeed1 1.5 > /dev/null &

./actEmergencyStop --lidarinscope /lidar --cntMax 15 --distance 0.25 --delay 10 --switchinscope /following > /dev/null &

./actTargetPosition --inscope /targetPositions > /dev/null &

./actAmiroMotor > /dev/null &

wait
cpufreq-set -g ondemand
