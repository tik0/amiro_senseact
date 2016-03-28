#!/bin/sh

./backgroundStop.sh

# start spread for local communication
spread &
# start spread for remote communication
spread -c amirospread &

sleep 5

./objectDetection -d 6 --loadingDirectly &

