cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

# Add folders to build and run CMakeLists.txt in it
add_subdirectory ($ENV{MUROX_PROJECT}/process/objectDelivery objectDelivery)
add_subdirectory ($ENV{MUROX_PROJECT}/process/stateMachineGEPTRO stateMachineGEPTRO)
add_subdirectory ($ENV{MUROX_PROJECT}/process/stateMachineGEPTRO_AMiRoEnvironmentSimulation amiroEnvironment)
add_subdirectory ($ENV{MUROX_PROJECT}/process/answerer_tobi answerer_tobi)
add_subdirectory ($ENV{MUROX_PROJECT}/process/drivingObjectDetection drivingObjectDetection)
add_subdirectory ($ENV{MUROX_PROJECT}/process/rirReader rirReader)
add_subdirectory ($ENV{MUROX_PROJECT}/act/stopAMiRo stopAMiRo)
add_subdirectory ($ENV{MUROX_PROJECT}/process/mapGenerator mapGenerator)
add_subdirectory ($ENV{MUROX_PROJECT}/process/frontierExploration frontierExploration)
add_subdirectory ($ENV{MUROX_PROJECT}/process/localPlannerISY localPlannerISY)
add_subdirectory ($ENV{MUROX_PROJECT}/process/motorControl motorControl)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseFloorProximity senseFloorProximity)
add_subdirectory ($ENV{MUROX_PROJECT}/process/objectDetection/objectDetection objectDetection)

