#!/bin/bash

(sleep 10; xterm -e sh -c './show-map.sh # > /dev/null & ./keyboard-control.sh; wait') &

ssh -t amiro21 'cd particlefilterEvaluation; ./run.sh'
wait

