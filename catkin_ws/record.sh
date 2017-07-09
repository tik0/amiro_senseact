#/bin/bash

rsbagcl0.16 record \
  --idl-path "${MUROX_INCLUDE_DIRS}/types/" \
  --load-idl "${MUROX_INCLUDE_DIRS}/types/loc.proto"  \
  --idl-path "/usr/share/rst0.16/proto/stable/" \
  --load-idl "/usr/share/rst0.16/proto/stable/rst/vision/LaserScan.proto" \
  --load-idl "/usr/share/rst0.16/proto/stable/rst/geometry/Pose.proto" \
  -o tmp.tide \
  'spread:/tracking/merger' \
  'spread:/amiro1/'
