cmake_minimum_required(VERSION 2.6)

# Set the project name to the name of the folder
include("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")

add_subdirectory("$ENV{MUROX_PROJECT}/process/localization/particlefilter" particlefilter)

add_subdirectory("$ENV{MUROX_PROJECT}/process/localization/CoreSLAMLocalization" CoreSLAMLocalization)

add_subdirectory("$ENV{MUROX_PROJECT}/process/actingBehavior/driveWaypoints" driveWaypoints)

#add_subdirectory("$ENV{MUROX_PROJECT}/act/actTargetPosition" actTargetPosition)

add_subdirectory("$ENV{MUROX_PROJECT}/act/actAmiroMotor" actAmiroMotor)

add_subdirectory("$ENV{MUROX_PROJECT}/sense/senseHokuyo" senseHokuyo)

add_subdirectory("$ENV{MUROX_PROJECT}/sense/sendOdometryProtoPose" sendOdometryProtoPose)

add_subdirectory("$ENV{MUROX_PROJECT}/tools/robotTools/stopAMiRo" stopAMiRo)
