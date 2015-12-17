cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

# Add folders to build and run CMakeLists.txt in it
add_subdirectory ($ENV{MUROX_PROJECT}/process/objectDelivery objectDelivery)
add_subdirectory ($ENV{MUROX_PROJECT}/process/stateMachineGEPTRO stateMachineGEPTRO)
add_subdirectory ($ENV{MUROX_PROJECT}/process/stateMachineGEPTRO_AMiRoEnvironmentSimulation amiroEnvironment)
add_subdirectory ($ENV{MUROX_PROJECT}/process/stateMachineGEPTRO_CommandSimulation commandSimulation)
add_subdirectory ($ENV{MUROX_PROJECT}/process/exploDriveEdge exploDriveEdge)
add_subdirectory ($ENV{MUROX_PROJECT}/process/drivingObjectDetection drivingObjectDetection)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseRingProximity senseRingProximity)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseOdometry senseOdometry)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseHokuyo senseHokuyo)
add_subdirectory ($ENV{MUROX_PROJECT}/process/CoreSLAMBasic CoreSLAM)
add_subdirectory ($ENV{MUROX_PROJECT}/process/localPlanner localPlanner)
add_subdirectory ($ENV{MUROX_PROJECT}/process/objectDetection/objectDetection objectDetection)
add_subdirectory ($ENV{MUROX_PROJECT}/tools/stopAMiRo stopAMiRo)

