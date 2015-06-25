#!/bin/sh
echo "Call this script with a songnumber described in start.txt"
TMPFILE="/tmp/rsbsend"
echo "song${1}" > ${TMPFILE}
rsb-sendcpp0.11 /choreo ${TMPFILE}
