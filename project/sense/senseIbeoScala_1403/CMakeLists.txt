cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")


# Read the pkg-config modules
FIND_PACKAGE(PkgConfig)
pkg_check_modules(RSB rsb0.11)
pkg_check_modules(RSC rsc0.11)
pkg_check_modules(PROTOBUF protobuf)
pkg_check_modules(RST rstsandbox0.11)

# Get modules
SET (BOOST_COMPONENTS regex date_time program_options system thread)
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/findPackageBoost.cmake")

# Get the compiler specific includes and store them in GXX_INCLUDES
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setCompilerIncludes.cmake")

find_library(IBEOSDK
libibeosdk.so
./IbeoSDK4.1.1/libs)

if(IBEOSDK)
message (STATUS "IBEO Library found")
endif()

# Set the include pathe
include_directories(
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    ${RST_INCLUDE_DIRS}                                             # RSC
    ${PROTOBUF_INCLUDE_DIRS}                                        # Protobuffer
    "$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
    ${CMAKE_CURRENT_BINARY_DIR}                                     # add the binary tree to the search path for
    ${GXX_INCLUDES}
    ${Boost_INCLUDE_DIR}
    "./IbeoSDK4.1.1/src"
)

# Define the source
SET (SOURCE
	$ENV{MUROX_INCLUDE_DIRS}/types/IbeoScala_1403.pb.cc
	$ENV{MUROX_INCLUDE_DIRS}/types/PoseEuler.pb.cc
	$ENV{MUROX_INCLUDE_DIRS}/types/RotationEuler.pb.cc
	src/IbeoScalaDataListener.cpp 
)

LINK_DIRECTORIES (
    ${RSB_LIBRARY_DIRS}
    ${RSC_LIBRARY_DIRS}
)

# add the executable
add_executable ("${PROJECT_NAME}" src/main.cxx ${SOURCE})

target_link_libraries (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${Boost_LIBRARIES}
    ${IBEOSDK}
)

# Special flags for RST
ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS} -Wall -Wno-deprecated-declarations -O2 -Wno-write-strings -std=c++11)

# Set compiler flags as you like
SET ( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wno-deprecated-declarations -O0 -g -Wno-write-strings -std=c++11" )
SET ( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -Wno-deprecated-declarations -O2 -Wno-write-strings -std=c++11" )
