cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

# Add folders to build and run CMakeLists.txt in it
add_subdirectory ($ENV{MUROX_PROJECT}/process/mapGenerator mapGenerator)
add_subdirectory ($ENV{MUROX_PROJECT}/process/frontierExploration frontierExploration)
add_subdirectory ($ENV{MUROX_PROJECT}/process/localPlannerISY localPlannerISY)
add_subdirectory ($ENV{MUROX_PROJECT}/process/motorControl motorControl)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseRingProximity senseRingProximity)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseFloorProximity senseFloorProximity)
add_subdirectory ($ENV{MUROX_PROJECT}/act/stopAMiRo stopAMiRo)
