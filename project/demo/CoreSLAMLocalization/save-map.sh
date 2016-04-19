#!/bin/bash
filename=${1:-map.pgm}
RSB_TRANSPORT_SPREAD_PORT=4823 rsb-sendcpp0.11 /saveMap <(echo "$filename")
