cmake_minimum_required (VERSION 2.6)

include($ENV{MUROX_PROJECT}/utilities/cmake/setProjectName.cmake)

# Add folders to build and run CMakeLists.txt in it
add_subdirectory ($ENV{MUROX_PROJECT}/process/localization/CoreSLAMLocalization CoreSLAM)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/sendOdometryProtoPose sendOdometryProtoPose)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseHokuyo senseHokuyo)
add_subdirectory ($ENV{MUROX_PROJECT}/act/actAmiroMotor actAmiroMotor)
add_subdirectory ($ENV{MUROX_PROJECT}/tools/robotTools/stopAMiRo stopAMiRo)
