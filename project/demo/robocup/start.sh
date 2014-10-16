#!/bin/bash
for ID in {0..9}; do
  echo "Send 'start' to ID ${ID}"
  rsb-sendcpp0.9 /tabletop/BB${ID}/start void
done
exit 0

# tiko@lappy:~/repositories/murox/project/sandbox/TemplateMatchingWithBeBotCam$ rsb-sendcpp0.9 /tabletop/BB2/standby void
