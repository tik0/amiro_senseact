cmake_minimum_required (VERSION 2.8)

# Set the project name to the name of the folder
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")

# Read the pkg-config modules
find_package(PkgConfig)
pkg_check_modules(RSB rsb0.11)
pkg_check_modules(RSC rsc0.11)
pkg_check_modules(RST rst0.11)
find_package(OpenCV)

# Get modules
SET (BOOST_COMPONENTS regex date_time program_options system)
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/findPackageBoost.cmake")

# Get the compiler specific includes and store them in GXX_INCLUDES
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setCompilerIncludes.cmake")

# Set the include pathe
include_directories(
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    ${RST_INCLUDE_DIRS}                                             # RST
    ${GLIB_INCLUDE_DIRS}                                            # glib
    "../../includes"                                                # Other includes like converters and stuff
    "$ENV{CVB}/include"                                               # Stemmer imaging
    "${CMAKE_CURRENT_BINARY_DIR}"                                   # add the binary tree to the search path for include files
    ${GXX_INCLUDES}
)

# Define the source
SET (SOURCE 
main.cxx
ArgHandler.cpp
)

SET (HEADERS
ArgHandler.hpp
)

# Add sources to build
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/iplImageConverterSource.cmake")

# Set the compiler
# SET (CMAKE_C_COMPILER $ENV{CPP})
# SET (CMAKE_CXX_COMPILER $ENV{CXX})

# Link the executable to the library. 
LINK_DIRECTORIES (
    ${RSB_LIBRARY_DIRS}
    ${RSC_LIBRARY_DIRS}
    ${RST_LIBRARY_DIRS}
    ${OpenCV_LIBRARY_DIRS}
    "$ENV{SDKTARGETSYSROOT}/usr/lib"
    "$ENV{CVB}/lib"                                             # Stemmer imaging
    "${CMAKE_CURRENT_BINARY_DIR}/../ARToolKit/lib"              # ARToolKit
)

# add the executable
add_executable ("${PROJECT_NAME}" ${SOURCE})

target_link_libraries (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${Boost_LIBARIES}
    ${OpenCV_LIBRARIES}
    # -- stemmer imaging
    :"$ENV{CVB}/lib/libCVCImg.so"
    :"$ENV{CVB}/lib/libCVCUtilities.so"
    :"$ENV{CVB}/lib/libCVCDriver.so"
    :"$ENV{CVB}/lib/libCVGenApi.so"
)

# Additional compiler flags
ADD_DEFINITIONS (-Wno-deprecated-declarations -O3 ${RST_CFLAGS})

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long -pedantic")
# Get rid of depricated warnings
SET (
    CMAKE_CXX_FLAGS
    "-Wno-deprecated-declarations -Wno-write-strings -O2 -std=c++11 -g"
)
