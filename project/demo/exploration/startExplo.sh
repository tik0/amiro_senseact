#!/bin/sh
./stopExplo.sh
id=0
host=localhost
port=4823

prox_obstacle=/rir_prox/obstacle/

#./edgeAvoidanceBehavior > /dev/null &
./senseFloorProximity &
./mapGenerator -r --id $id --host $host --port $port --irin $prox_obstacle &
./motorControl >/dev/null  &
./senseRingProximity  &
./localPlannerISY --id $id --host $host --port $port &
./frontierExploration --id $id --host $host --port $port --irin $prox_obstacle &
sleep 2
./rsbsend.sh /exploration start
