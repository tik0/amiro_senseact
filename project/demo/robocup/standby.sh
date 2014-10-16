#!/bin/bash
for ID in {0..9}; do
  echo "Send 'standby' to ID ${ID}"
  rsb-sendcpp0.9 /tabletop/BB${ID}/standby void
done
exit 0
