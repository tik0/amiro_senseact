#!/bin/bash
export RSB_TRANSPORT_SPREAD_HOST=${1:-10.0.0.21}

$MUROX_PROJECT/tools/hmi/sendControlsVecInt/sendControlsVecInt -o /setTargetPosition
