#!/bin/bash
export RSB_TRANSPORT_SOCKET_ENABLED=0
export RSB_TRANSPORT_SPREAD_PORT=4823
export RSB_TRANSPORT_SPREAD_HOST=localhost
export RSB_TRANSPORT_SPREAD_ENABLED=1

rsb-sendcpp0.11 /sendMap <(echo enable)
../../tools/hmi/setPosition/setPosition --i /CoreSLAMLocalization/image/$1 --flipHorizontal --o /setPosition/$1
rsb-sendcpp0.11 /sendMap <(echo disable)
