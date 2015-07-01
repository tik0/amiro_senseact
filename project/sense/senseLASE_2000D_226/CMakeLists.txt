cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

project ("${PROJECT_NAME}")


# Read the pkg-config modules
find_package(PkgConfig)

pkg_check_modules(RSB rsb0.11)
pkg_check_modules(RSC rsc0.11)
pkg_check_modules(PROTO protobuf)

# Get the system includes
execute_process(COMMAND echo COMMAND ${CMAKE_CXX_COMPILER} -Wp,-v -x c++ - -fsyntax-only ERROR_VARIABLE GXX_OUTPUT)
set(ENV{GXX_OUTPUT} ${GXX_OUTPUT})
execute_process(COMMAND echo ${GXX_OUTPUT} COMMAND grep "^\ " COMMAND sed "s#\ ##g" COMMAND tr "\n" "\\;" OUTPUT_VARIABLE GXX_INCLUDES) 

set(source_files
  lase_interface.cpp
  $ENV{MUROX_INCLUDE_DIRS}/types/lase_2000D_226.pb.cc
  $ENV{MUROX_PROJECT}/tools/GenericInformer/src/GenericInformer.hpp
  $ENV{MUROX_INCLUDE_DIRS}/types/SensorPosition.pb.cc)

# Set the include pathe
include_directories(
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    ${PROTO_INCLUDE_DIRS}                                           # Protobuffer
    "$ENV{MUROX_INCLUDE_DIRS}"  
    "$ENV{MUROX_INCLUDE_DIRS}/types"                                          # Other includes like converters and stuff
    "$ENV{MUROX_PROJECT}/tools/GenericInformer/src"
    "${CMAKE_CURRENT_BINARY_DIR}"                                   # add the binary tree to the search path for
    ${GXX_INCLUDES}
)

# add the executable
add_executable ("${PROJECT_NAME}" main.cpp ${source_files})

# Link the executable to the library. 
# SET (BLAH "$ENV{SDKTARGETSYSROOT}/usr/lib")
link_directories (
    "$ENV{SDKTARGETSYSROOT}/usr/lib"
)
target_link_libraries (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${PROTO_LIBRARIES}
    boost_regex
    boost_date_time
    boost_program_options
    boost_system
    boost_thread
)

# Additional compiler flags
SET (CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wno-deprecated-declarations -O2 -Wno-write-strings")
