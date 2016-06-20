#!/bin/sh
cpufreq-set -g performance

# external spread
spread -c amirospread &
sleep 5

# internal spread
spread &
sleep 5

./CoreSLAM --odominscope /odom --lidarinscope /lidar --senImage 1 --delta 0.01 --sigma_xy 10 --sigma_xy_new_position 100 --sigma_theta 0.05 --sigma_theta_new_position 0.1 --remotePort 4823 --doMapUpdate true --hominginscope /homing &

if [ -e /dev/ttyACM0 ]; then
	./senseHokuyo -o /lidar &
else
	./senseSickTim -o /lidar &
fi

./sendOdometryProtoPose --resetodom true -o /odom &
./actTargetPosition --inscope /targetPositions & 
./actAmiroMotor -i /motor &

wait
cpufreq-set -g ondemand
