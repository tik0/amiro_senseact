cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")

# Check for build types and set the tty-outputs accordingly
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/chkBuildTypes.cmake")

# Read the pkg-config modules
FIND_PACKAGE(PkgConfig)
if ("$ENV{ARCH}" MATCHES arm)
  pkg_check_modules(RSB rsb0.11)
  pkg_check_modules(RSC rsc0.11)
  pkg_check_modules(PROTOBUF protobuf)
  pkg_check_modules(RST rstsandbox0.11)
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
    ${RSC_INCLUDE_DIRS}                                             # RSC
    "$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
    ${CMAKE_CURRENT_BINARY_DIR}                                     # add the binary tree to the search path for
    ${GXX_INCLUDES}
    ${Boost_INCLUDE_DIR}
)

# Define the source
SET (SOURCE main.cxx)

# Add sources to build
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/vecIntConverterSource.cmake")

# add the executable
ADD_EXECUTABLE ("${PROJECT_NAME}" ${SOURCE})

TARGET_LINK_LIBRARIES (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${Boost_LIBRARIES}
)

# Additional compiler flags
ADD_DEFINITIONS (-Wall -Wno-deprecated-declarations -O2 -Wno-write-strings -std=c++11)
