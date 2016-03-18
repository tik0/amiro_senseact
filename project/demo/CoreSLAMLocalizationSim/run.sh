#!/bin/bash

# Start the spread communication deamon
spread -c spreadSim.conf &
sleep 5
# Start the simulator
gazebo enclosed_world-no-cylinder.world &
# Start the SLAM program
./CoreSLAMLocalization/CoreSLAMLocalization --odominscope /AMiRo_Hokuyo/odom/ --lidarinscope /AMiRo_Hokuyo/lidar/ --senImage 1 --delta 0.01 --sigma_theta 0.18 --hole_width 0.2 --doMapUpdate false &
#./showCamJpg/showCamJpg -i /image &
echo "Enabled steering by wasd, enter (resend), space (halt)"
./sendTwistControls/sendTwistControls -o /AMiRo_Hokuyo/diffKin
