cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

# Add folders to build and run CMakeLists.txt in it
add_subdirectory ($ENV{MUROX_PROJECT}/process/actingBehavior/objectDelivery objectDelivery)
add_subdirectory ($ENV{MUROX_PROJECT}/process/statemachines/stateMachineGEPTRO stateMachineGEPTRO)
add_subdirectory ($ENV{MUROX_PROJECT}/process/statemachines/stateMachineGEPTRO_AMiRoEnvironmentSimulation amiroEnvironment)
add_subdirectory ($ENV{MUROX_PROJECT}/process/statemachines/stateMachineGEPTRO_CommandSimulation commandSimulation)
add_subdirectory ($ENV{MUROX_PROJECT}/process/exploration/exploDriveEdge exploDriveEdge)
add_subdirectory ($ENV{MUROX_PROJECT}/process/statemachines/drivingObjectDetection drivingObjectDetection)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseRingProximity senseRingProximity)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/sendOdometryProtoPose sendOdometryProtoPose)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseHokuyo senseHokuyo)
add_subdirectory ($ENV{MUROX_PROJECT}/process/mapping/CoreSLAMBasic CoreSLAM)
add_subdirectory ($ENV{MUROX_PROJECT}/process/exploration/localPlanner localPlanner)
add_subdirectory ($ENV{MUROX_PROJECT}/process/objectDetection/objectDetection/objectDetection objectDetection)
add_subdirectory ($ENV{MUROX_PROJECT}/tools/robotTools/stopAMiRo stopAMiRo)

