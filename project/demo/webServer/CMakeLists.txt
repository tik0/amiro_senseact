cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

# Add folders to build and run CMakeLists.txt in it
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseHokuyo senseHokuyo)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseSickTim senseSick)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseCamJpg senseCamJpg)
add_subdirectory ($ENV{MUROX_PROJECT}/sandbox/readRGBDCam senseRGBD)
add_subdirectory ($ENV{MUROX_PROJECT}/process/misc/rsb_ws_bridge_amiro rsb_ws_bridge_amiro)
