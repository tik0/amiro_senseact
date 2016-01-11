#!/bin/bash
echo "Start localization with camera ${1}"
cp GenICam${1}.ini /etc/opt/cvb/drivers/GenICam.ini
./localizationTwb \
--record 0 \
--confidenceFactor 0.5 \
--tresh 150 \
--outscope /tracking/${1} \
--publishCounter 1 \
--onlyPublishIfMarkerDetected 0 \
--camGainAbs 25 \
--camReverseX 0 \
--camExposureTimeRaw 16000 \
--camNoiseCorrection 1 \
--camDefectCorrection 0 \
--camNegativeImage 0 \
--camAcquisitionFrameRateAbs 15
