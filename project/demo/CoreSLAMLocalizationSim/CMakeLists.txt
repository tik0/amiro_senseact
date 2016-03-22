cmake_minimum_required (VERSION 2.6)

INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")

# Add folders to build and run CMakeLists.txt in it
add_subdirectory ($ENV{MUROX_PROJECT}/process/localization/CoreSLAMLocalization CoreSLAMLocalization)
add_subdirectory ($ENV{MUROX_PROJECT}/tools/hmi/sendControlsProtoTwist sendTwistControls)
#add_subdirectory ($ENV{MUROX_PROJECT}/tools/hmi/showCamJpg showCamJpg)
