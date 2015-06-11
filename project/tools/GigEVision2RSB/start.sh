#!/bin/bash
echo "Start camera ${1}"
#cp GenICam${1}.ini /etc/opt/cvb/drivers/GenICam.ini
#start file for manta camera
./GigEVision2RSB \
--record 0 \
--outscope /gigevision/cam \
--recordFilename video.avi \
--numFrames 0 \
--camStreamBytesPerSecond 12400000 \
--camPixelFormat 1 \
--camWidth 500 \
--camHeight 250 \
--camOffsetX 0 \
--camOffsetY 0 \
--camExposureTimeAbs 16000 \
--camExposureAuto 0 \
--camGain 0.00 \
--camGainAuto 0 \
--camBlackLevel 4.00 \
--camGamma 1.00 \


#optional
#--fpsRecord 1 \
