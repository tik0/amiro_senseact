cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

# Add folders to build and run CMakeLists.txt in it
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseOdometry senseOdometry)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseHokuyo senseHokuyo)
add_subdirectory ($ENV{MUROX_PROJECT}/act/actAmiroMotor actAmiroMotor)
add_subdirectory ($ENV{MUROX_PROJECT}/process/mapping/CoreSLAMBasic CoreSLAM)
