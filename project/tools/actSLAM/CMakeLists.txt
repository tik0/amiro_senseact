cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")

# Get modules
SET (BOOST_COMPONENTS regex date_time program_options system thread)
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/findPackageBoost.cmake")

# Get the compiler specific includes and store them in GXX_INCLUDES
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setCompilerIncludes.cmake")

find_package(RSB REQUIRED)
find_package(RSC REQUIRED)
find_package(RST 0.11 REQUIRED COMPONENTS sandbox)
find_package(Protobuf REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

# Set the include pathe
include_directories(BEFORE SYSTEM ${RST_INCLUDE_DIRS})
include_directories(include
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    ${RST_INCLUDE_DIRS}                                             # RSC
    ${PROTOBUF_INCLUDE_DIRS}                                        # Protobuffer
    "$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
    ${CMAKE_CURRENT_BINARY_DIR}                                     # add the binary tree to the search path for
    ${GXX_INCLUDES}
    ${Boost_INCLUDE_DIR}
    "$ENV{MUROX_PROJECT}/tools/tinyslam/trunk"
)

# Define the source
SET (SOURCE main.cxx)

# add the executable
ADD_EXECUTABLE ("${PROJECT_NAME}" ${SOURCE})
# ADD_EXECUTABLE ("${PROJECT_NAME}" ${SOURCE} CoreSLAM.c CoreSLAM_loop_closing.c CoreSLAM_state.c CoreSLAM_random.c CoreSLAM_ext.c CoreSLAM.h)

# Link the executable to the library.
# target_link_directories (
#     "/usr/lib"                             # For standard libraries
#     "/usr/local/lib/"                      # For OpenCV libraries
#     "/usr/lib/x86_64-linux-gnu/"           # For boost libraries
# )

FIND_LIBRARY(CORESLAM_LIBRARY NAMES libCoreSLAM.a PATHS $ENV{MUROX_PROJECT}/tools/tinyslam/build)

# message(STATUS "CORESLAM ${CORESLAM_LIBRARY}")

set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} $ENV{MUROX_PROJECT}/tools/tinyslam/build)

# get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
#     message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()

LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})
link_directories($ENV{MUROX_PROJECT}/tools/tinyslam/install/lib)

TARGET_LINK_LIBRARIES (
    "${PROJECT_NAME}"
    :${CORESLAM_LIBRARY}
#     :/opt/repositories/murox_dev/project/tinyslam/trunk/libCoreSLAM.a
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
)

# Additional compiler flags
ADD_DEFINITIONS (-Wall -Wno-deprecated-declarations -O0 -g -Wno-write-strings -std=c++11)
# Special flags for RST
ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})