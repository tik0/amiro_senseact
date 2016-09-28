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

# sensing lidar
if [ -e /dev/ttyACM0 ]; then
	echo "Assuming Hokuyo laser scanner"
	./senseHokuyo -d /dev/ttyACM0 -o /lidar > /dev/null &
else
	echo "Assuming SICK laser scanner"
	./senseSickTim -o /lidar > /dev/null &
fi

# waypoint program
sleep 1 # sleep so we don't get old laser values
./waypoint --lidarinscope /lidar --range 1.5 --scanStartIndex 390 --scanEndIndex 420 -s > /dev/null &

# start state machine
poseXThresholds="19.0
17.0"

poseXThreshold="$(echo "$poseXThresholds" | head -n $((${ID} + 1 % 4)) | tail -n 1)"
./final2016 --id ${ID} --poseScope /amiro${ID}/pose --poseXThreshold $poseXThreshold > /dev/null &

# sense odometry data
./sendOdometryProtoPose --resetodom true -o /odom > /dev/null &

# start follower
./follow_LaserScanner -l /lidar --forwardSpeed 800 --forwardMinSpeed 400 --followMinDist 400 --followDistSlowingDown 400 --followMinBackDist 200 --maxRange 1500 --turningSpeed 50 &

# initial poses for CoreSLAM
initialPoses="17677.1 19037.1 -45
16467.2 18082.7 45"

initialPose="$(echo "$initialPoses" | head -n $((${ID} + 1 % 4)) | tail -n 1)"
initialX="$(echo "$initialPose" | cut -d\  -f 1)"
initialY="$(echo "$initialPose" | cut -d\  -f 2)"
initialTheta="$(echo "$initialPose" | cut -d\  -f 3)"

./CoreSLAM --odominscope /odom --lidarinscope /lidar --hominginscope /homing --mapAsImageOutScope /CoreSLAMLocalization/image --setPositionScope /setPosition/${ID} --targetPoseInScope /setTargetPose/${ID} --positionOutScope /amiro${ID}/pose --pathOutScope /amiro${ID}/path \
  --remotePort 4823 \
  --senImage 0 \
  --delta 0.05 --sigma_xy 10 --sigma_xy_new_position 100 --sigma_theta 0.1 --sigma_theta_new_position 0.15  --doMapUpdate false \
  --loadMapFromImage ./data/homecomingCitec.pgm --flipHorizontal true \
  --erosionRadius 0.35 \
  --initialX $initialX --initialY $initialY --initialTheta $initialTheta \
  --precomputeOccupancyMap true \
  --targetSpeed0 0 --targetSpeed1 1.5 &

./actEmergencyStop --lidarinscope /lidar --cntMax 15 --distance 0.25 --delay 10 --switchinscope /following > /dev/null &

./actTargetPosition --inscope /targetPositions > /dev/null &

./actAmiroMotor > /dev/null &

wait
cpufreq-set -g ondemand
