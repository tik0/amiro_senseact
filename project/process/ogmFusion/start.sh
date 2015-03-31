#!/bin/bash
killall ogmFusion
./ogmFusion \
--iEdge  /AMiRo_Hokuyo_Pitch_9/ogmEdge \
--iWeed  /AMiRo_Hokuyo_Pitch_9/ogmWeed \
--iCrop  /AMiRo_Hokuyo_Pitch_9/ogmCrop \
--iFloor /AMiRo_Hokuyo_Pitch_9/ogmFloor
