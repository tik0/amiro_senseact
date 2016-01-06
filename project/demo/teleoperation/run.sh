#!/bin/sh
cpufreq-set -g performance
spread &
sleep 5
./teleoperationWebserver --resource_path ./Server --port 7681 --thread_camera &
./actAmiroMotor -i /STEERING &
./senseRingProximity -o /IR --noObstacleValues --noEdgeValues &
wait
cpufreq-set -g ondemand
