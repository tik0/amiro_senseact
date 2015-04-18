#!/bin/bash

# Start the spread communication deamon
spread -c spreadSim.conf &
sleep 5
# Start the simulator
gazebo &
# Start the SLAM program
./CoreSLAM/CoreSLAM --odominscope /AMiRo_Hokuyo_tilted/gps/ --lidarinscope /AMiRo_Hokuyo_tilted/lidar/ --senImage 1 --delta 0.01 --rotY 30 &
echo "Enabled steering by wasd, enter (resend), space (halt)"
./sendTwistControls/sendTwistControls -o /AMiRo_Hokuyo_tilted/diffKin
