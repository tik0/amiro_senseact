#!/bin/sh
cpufreq-set -g performance

# external spread
spread -c amirospread &
sleep 5

# internal spread
spread &
sleep 5

./CoreSLAM --odominscope /odom --lidarinscope /lidar --senImage 1 --delta 0.01 --sigma_xy 0.1 --sigma_theta 0.05 --remotePort 4823 --doMapUpdate true --loadMapWithValidPositionsFromPNG ./data/arenaEindhovenValid.png --loadMapFromImage ./data/arenaEindhoven.png --hominginscope /homing &
./senseHokuyo -o /lidar &
./sendOdometryProtoPose --resetodom true -o /odom &
./actTargetPosition --inscope /targetPositions & 
RSB_TRANSPORT_SPREAD_PORT=4823 ./actAmiroMotor -i /motor &

wait
cpufreq-set -g ondemand
