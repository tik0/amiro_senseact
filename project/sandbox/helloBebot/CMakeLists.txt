cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")

# Get the compiler specific includes and store them in GXX_INCLUDES
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setCompilerIncludes.cmake")

# Set the include pathe
include_directories(
    ${GXX_INCLUDES}
)

# Define the source
SET (SOURCE main.cxx)

# add the executable
ADD_EXECUTABLE ("${PROJECT_NAME}" ${SOURCE})

# Additional compiler flags
ADD_DEFINITIONS (-std=c++0x -Wall -Wno-deprecated-declarations -O2 -Wno-write-strings)
