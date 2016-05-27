#!/bin/bash
export RSB_TRANSPORT_SPREAD_HOST=${1:-192.168.2.37}

filename=${2:-map.pgm}
rsb-sendcpp0.11 /saveMap <(echo "$filename")
