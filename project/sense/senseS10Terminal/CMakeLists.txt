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

# Set the include pathe
include_directories(
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    ${RST_INCLUDE_DIRS}                                             # RSC
    ${PROTOBUF_INCLUDE_DIRS}                                        # Protobuffer
    "$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
    "nmealib/include"
    ${CMAKE_CURRENT_BINARY_DIR}                                     # add the binary tree to the search path for
    ${GXX_INCLUDES}
    ${Boost_INCLUDE_DIR}
)

set(source_files
  src/rs232.c
  nmealib/src/context.c
  nmealib/src/generate.c
  nmealib/src/generator.c
  nmealib/src/gmath.c
  nmealib/src/info.c
  nmealib/src/parse.c
  nmealib/src/parser.c
  nmealib/src/sentence.c
  nmealib/src/time.c
  nmealib/src/tok.c
  $ENV{MUROX_INCLUDE_DIRS}/types/Nmea.pb.cc
)

# Define the source
SET (SOURCE src/main.cxx ${source_files})
#SET (SOURCE nmealib/samples/parse/main.c ${source_files})

LINK_DIRECTORIES (
    ${RSB_LIBRARY_DIRS}
    ${RSC_LIBRARY_DIRS}
)

# add the executable
add_executable ("${PROJECT_NAME}" ${SOURCE})

target_link_libraries (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${Boost_LIBRARIES}
)

# Special flags for RST
ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})

# Set compiler flags as you like
SET ( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wno-deprecated-declarations -O0 -g -Wno-write-strings -std=c++11" )
SET ( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -Wno-deprecated-declarations -O2 -Wno-write-strings -std=c++11" )
