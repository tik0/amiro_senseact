#!/bin/bash
export RSB_TRANSPORT_SPREAD_HOST=${1:-192.168.2.37}

rsb-sendcpp0.11 /homing <(echo homing)