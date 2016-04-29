cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")

if("$ENV{ARCH}" MATCHES arm)
# Read the pkg-config modules
FIND_PACKAGE(PkgConfig)
pkg_check_modules(RSB rsb0.11)
pkg_check_modules(RSC rsc0.11)
pkg_check_modules(RST rstsandbox0.11)
pkg_check_modules(PROTOBUF protobuf)

else("$ENV{ARCH}" MATCHES arm)

find_package(RSB REQUIRED)
find_package(RSC REQUIRED)
find_package(RST COMPONENTS sandbox)
find_package(Protobuf REQUIRED)

endif("$ENV{ARCH}" MATCHES arm)

# Get modules
SET (BOOST_COMPONENTS regex date_time program_options system thread)
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/findPackageBoost.cmake")

# Get the compiler specific includes and store them in GXX_INCLUDES
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setCompilerIncludes.cmake")

# Set the include pathe
include_directories(
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RST_INCLUDE_DIRS}                                             # RST
    ${RSC_INCLUDE_DIRS}                                             # RSC
    "$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
    ${CMAKE_CURRENT_BINARY_DIR}                                     # add the binary tree to the search path for
    ${GXX_INCLUDES}
    ${Boost_INCLUDE_DIR}
    ${PROTOBUF_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}
)

# Define the source
SET (SOURCE main.cxx "$ENV{MUROX_INCLUDE_DIRS}/types/TargetPoseEuler.pb.cc" "$ENV{MUROX_INCLUDE_DIRS}/types/PoseEuler.pb.cc" "$ENV{MUROX_INCLUDE_DIRS}/types/RotationEuler.pb.cc")

# add the executable
ADD_EXECUTABLE ("${PROJECT_NAME}" ${SOURCE})

TARGET_LINK_LIBRARIES (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
)

# Additional compiler flags
SET ( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wno-deprecated-declarations -O0 -g -Wno-write-strings -std=c++11" )
SET ( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -Wno-deprecated-declarations -O2 -Wno-write-strings -std=c++11" )

# Special flags for RST
ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})