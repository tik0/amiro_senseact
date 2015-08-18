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
--outscope /sense/gigevision/manta \
--recordFilename video.avi \
--numFrames 0 \
--camStreamBytesPerSecond 124000000 \
--camPixelFormat 17301505 \
--camWidth 2048 \
--camHeight 1088 \
--camOffsetX 0 \
--camOffsetY 0 \
--camExposureTimeAbs 16000 \
--camExposureAuto 4 \
--camGain 0.00 \
--camGainAuto 3 \
--camBlackLevel 4.00 \
--camGamma 1.00 \
--camTriggerMode Off \
--camAcquisitionFrameRateAbs 20.0 \

#optional
#--fpsRecord 1 \
