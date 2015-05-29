#!/bin/bash
./killChallenge.sh
spread -c amirospread &
sleep 5

./senseHokuyo -d /dev/ttyACM0 -o /lidar &
./objectDetectionAMiRo -d6 &
./openChallenge2015 -i /tobiamiro1 --outscopeStateTobi /amiro1tobi &
