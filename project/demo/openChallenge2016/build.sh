#!/bin/bash
cmake -DCMAKE_BUILD_TYPE=Release . && make $@
code=$?
notify-send "Build terminated"
exit $code
