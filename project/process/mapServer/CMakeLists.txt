cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")

# Get modules
SET (BOOST_COMPONENTS regex date_time program_options system thread)
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/findPackageBoost.cmake")

# Get the compiler specific includes and store them in GXX_INCLUDES
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setCompilerIncludes.cmake")

# Read the pkg-config modules
FIND_PACKAGE(PkgConfig)
pkg_check_modules(RSB rsb0.13)
pkg_check_modules(RSC rsc0.13)
pkg_check_modules(PROTOBUF protobuf)
pkg_check_modules(RST rstsandbox0.11)
pkg_check_modules(OpenCV opencv)
pkg_check_modules(EIGEN eigen3)
pkg_check_modules(MRPT mrpt-base mrpt-maps mrpt-obs mrpt-slam mrpt-gui)
# FIND_PACKAGE(OpenCV REQUIRED)
# FIND_PACKAGE(RSB REQUIRED)
# FIND_PACKAGE(RSC REQUIRED)
#  FIND_PACKAGE(RST REQUIRED COMPONENTS sandbox)

# Set the include pathe
include_directories(BEFORE SYSTEM ${RST_INCLUDE_DIRS})
include_directories(include
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    ${RST_INCLUDE_DIRS}                                             # RST
    ${PROTOBUF_INCLUDE_DIRS}                                        # Protobuffer
    "$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
    ${CMAKE_CURRENT_BINARY_DIR}                                     # add the binary tree to the search path for
    ${GXX_INCLUDES}
    ${MRPT_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

# Define the source
SET (SOURCE main.cxx $ENV{MUROX_INCLUDE_DIRS}/types/MachineModel.pb.cc)

ADD_EXECUTABLE ("${PROJECT_NAME}" ${SOURCE})

MESSAGE(INFO "AAAAAAAAAAAAA ${MRPT_LIBRARIES}")
TARGET_LINK_LIBRARIES (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${MRPT_LIBRARIES}
)

# Additional compiler flags
SET ( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wno-deprecated-declarations -O0 -g -Wno-write-strings -std=c++11 -funsafe-math-optimizations -Wno-unused-variable" )
SET ( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -Wno-deprecated-declarations -O2 -Wno-write-strings -std=c++11 -funsafe-math-optimizations -Wno-unused-variable" )
# Special flags for RST
ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})
