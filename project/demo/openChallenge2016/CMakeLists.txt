cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

# Add folders to build and run CMakeLists.txt in it
add_subdirectory ($ENV{MUROX_PROJECT}/sense/sendOdometryProtoPose sendOdometryProtoPose)
add_subdirectory ($ENV{MUROX_PROJECT}/process/statemachines/final2016 final2016)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/waypoint waypoint)
add_subdirectory ($ENV{MUROX_PROJECT}/process/actingBehavior/follow_LaserScanner follow_LaserScanner)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseHokuyo senseHokuyo)
add_subdirectory ($ENV{MUROX_PROJECT}/tools/robotTools/stopAMiRo stopAMiRo)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseCamJpg senseCamJpg)
add_subdirectory ($ENV{MUROX_PROJECT}/process/misc/rsb_ws_bridge_amiro rsb_ws_bridge_amiro)
add_subdirectory ($ENV{MUROX_PROJECT}/process/localization/CoreSLAMLocalization CoreSLAM)
add_subdirectory ($ENV{MUROX_PROJECT}/act/actTargetPosition actTargetPosition)
add_subdirectory ($ENV{MUROX_PROJECT}/act/actEmergencyStop actEmergencyStop)