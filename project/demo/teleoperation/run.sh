#!/bin/sh
cpufreq-set -g performance
spread &
sleep 5
./websocketServerLibWebSockets --resource_path ./Server --port 7681 --thread_camera &
./actAmiroMotor -i /STEERING &
./senseRingProximity -o /IR &
wait
cpufreq-set -g ondemand
