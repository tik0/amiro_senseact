#!/bin/bash
export RSB_TRANSPORT_SPREAD_PORT=4823

rsb-sendcpp0.11 /sendMap <(echo enable)
../../tools/hmi/setPosition/setPosition --i /CoreSLAMLocalization/image --flipHorizontal
rsb-sendcpp0.11 /sendMap <(echo disable)
