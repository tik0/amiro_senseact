#!/bin/bash
echo "Start camera ${1}"

#note:
#camExposureAuto: range from 1-4
## 1 = Off
## 2 = Continous
## 3 = Off (maybe "Once", but after it is "Off" again)
## 4 = other (idk what this is)
#camGainAuto: range from 1-3
## 1 = Off
## 2 = Continous
## 3 = Off (maybe "Once", but after it is "Off" again)

#start file for manta camera
./GigEVision2RSB \
--record 0 \
--outscope /gigevision/cam \
--recordFilename video.avi \
--numFrames 0 \
--camAcquisitionFrameRateAbs 30

#optional
#--fpsRecord 1 \
