cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

# Add folders to build and run CMakeLists.txt in it
add_subdirectory ($ENV{MUROX_PROJECT}/process/statemachines/final2015 final2015)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/waypoint waypoint)
add_subdirectory ($ENV{MUROX_PROJECT}/process/follow_LaserScanner follow_LaserScanner)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseHokuyo senseHokuyo)
add_subdirectory ($ENV{MUROX_PROJECT}/tools/stopAMiRo stopAMiRo)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseCamJpg senseCamJpg)
add_subdirectory ($ENV{MUROX_PROJECT}/tools/rsb_ws_bridge_amiro rsb_ws_bridge_amiro)
