#!/bin/bash

# Start the spread communication deamon
spread -c spreadSim.conf &
sleep 5
# Start the simulator
gazebo myWorld.world &
# Start the SLAM program
./CoreSLAM/CoreSLAM --odominscope /AMiRo_Hokuyo/gps/ --lidarinscope /AMiRo_Hokuyo/lidar/ --senImage 1 --delta 0.01 &
./showCamJpg/showCamJpg -i /image &
echo "Enabled steering by wasd, enter (resend), space (halt)"
./sendTwistControls/sendTwistControls -o /AMiRo_Hokuyo/diffKin
