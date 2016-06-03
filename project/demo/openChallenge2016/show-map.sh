#!/bin/bash
# adapt host to AMiRo's IP
export RSB_TRANSPORT_SPREAD_HOST=${1:-192.168.2.37}

rsb-sendcpp0.11 /sendMap <(echo enable)
../../tools/hmi/setPosition/setPosition --i /CoreSLAMLocalization/image --flipHorizontal --o /setPosition
rsb-sendcpp0.11 /sendMap <(echo disable)
