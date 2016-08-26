#!/bin/sh

# process arguments
trackingId=${1:-12}

echo "tracking id: $trackingId"

# make sure everything is stopped afterwards
trap 'echo got SIGINT or script is exiting; ./stop.sh; exit' EXIT SIGINT

./stop.sh

# RSB scopes
lidarscope='/AMiRo_Hokuyo/lidar'
odomscope='/AMiRo_Hokuyo/odom'
kinscope='/AMiRo_Hokuyo/diffKin'
imagescope='/image'
poseestimatescope='/setPosition' # scope for setting the pose
posescope='/estimated_pose' # scope on which the estimated pose is published
motorCmdScope='/setTargetPosition'

is_spread_running() {
    if kill -0 $1; then
        echo "spread is running"
        return 0
    else
        echo "spread is not running"
        rm /tmp/4803
        return 1
    fi
}

# start spread
#while true; do
#    spread &
#    SPREAD_PID=$!
#    sleep 5
#
#    if is_spread_running $SPREAD_PID; then
#        break;
#    else
#        echo "spread not running. retrying..."
#    fi
#done

# firstly we need some input from a laser scanner
./senseHokuyo --outscope "$lidarscope" > /dev/null 2> /dev/null &

# and odometry
./sendOdometryProtoPose --outscope "$odomscope" --resetodom true > /dev/null 2> /dev/null &

# and acting
#./actTargetPosition --inscope "$motorCmdScope" &
./actAmiroMotor --inscope "$motorCmdScope"  > /dev/null 2> /dev/null &

# in TWB coordinates
START_POSITION="-1.01784 -1.88727 1.597391654" # 91.5238 <- degrees

# start position in pixel coordinates:
# 337, 197                  // * meter per pixel
# 2.271910101, 1.328089881  // to millimeter
CORESLAM_INITIAL_X="2271.910101"
CORESLAM_INITIAL_Y="1328.089881"
CORESLAM_INITIAL_THETA="91.5238"

PARTICLEFILTER_INITIAL_X="2.271910101"
PARTICLEFILTER_INITIAL_Y="1.328089881"
PARTICLEFILTER_INITIAL_THETA="1.597391654"

# start particle filter
./particlefilter \
  --lidarInScope "$lidarscope" --odomInScope "$odomscope" --poseEstimateOutScope "$posescope" \
  --sampleCount 500 \
  --newSampleProb 0 \
  --beamskip 1 \
  --debugImageOutScope /image \
  --initialPose "$PARTICLEFILTER_INITIAL_X" "$PARTICLEFILTER_INITIAL_Y" "$PARTICLEFILTER_INITIAL_THETA" \
  --pathToMap ./visualizationTwb_screenshot_24.08.2016-flip.png --meterPerPixel 0.006741573 \
#  --pathToMap ./twb-2016-08-23-D.png --meterPerPixel 0.01 \


./stop.sh
exit 1

# now everything is setup and we can start with the evaluation
iterations=20

PI_2='1.57079632679'
PI='3.14159265359'

# waypoint are relative
WAYPOINTS="
--waypoint 0 1.20 0
--waypoint -1.20 1.20 $PI_2
--waypoint -1.20 0 $PI
--waypoint 0 0 0
"


# 0.0349 rad = 2 deg;
navigationParameters="--yawTolerance 0.1 --xyTolerance 0.03 --sleepTime 200"

for i in `seq $iterations`; do
    # stop old localization
    killall CoreSLAMLocalization

    # first drive to the starting position by TWB tracking
    ./driveWaypoints --trackingId "$trackingId" --motorCmdOutScope "$motorCmdScope" --motorCommand setTargetSpeed \
            --waypoint $START_POSITION \
            --poseLog "./start-iteration-$i.log" \
            $navigationParameters

    echo "reached start position"

    # stop old localization and reset odom
    killall CoreSLAMLocalization
    killall sendOdometryProtoPose

    # shortly wait here
    sleep 5

    # start the localization
    ./sendOdometryProtoPose --outscope "$odomscope" --resetodom true > /dev/null 2> /dev/null &
    ./CoreSLAMLocalization \
        --odominscope "$odomscope" --lidarinscope "$lidarscope" \
        --delta 0.006741573 --loadMapFromImage ./visualizationTwb_screenshot_24.08.2016-flip.png \
        --sigma_xy 10 --sigma_theta 0.05 --sigma_xy_new_position 100 --sigma_theta_new_position 5 \
        --senImage 1 --mapAsImageOutScope /image --setPositionScope "/thisonedoesnotexist" \
        --positionOutScope "$posescope" --publishPoseEuler \
        --initialX "$CORESLAM_INITIAL_X" --initialY "$CORESLAM_INITIAL_Y" --initialTheta "$CORESLAM_INITIAL_THETA" &

    echo "started localization"

    # wait for localization to be ready
    sleep 10

    # then drive a rectangle
    ./driveWaypoints --poseInScope "$posescope" --motorCmdOutScope "$motorCmdScope" --motorCommand setTargetSpeed \
        --poseLog "./coreslam-iteration-$i.log" \
        $navigationParameters \
        $WAYPOINTS --relative

    # wait briefly to signal that this iteration is finished
    sleep 5

    echo "iteration $i done"
done

# lastly drive to the starting position by TWB tracking
# so that we can also evaluate the last iteration
./driveWaypoints --trackingId "$trackingId" --motorCmdOutScope "$motorCmdScope" --motorCommand setTargetSpeed \
        --waypoint $START_POSITION \
        --poseLog "./final.log" \
        $navigationParameters

echo ""
echo "==== stopping ===="
echo ""

./stop.sh

