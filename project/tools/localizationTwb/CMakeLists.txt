cmake_minimum_required (VERSION 2.8)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

project ("${PROJECT_NAME}")


# Read the pkg-config modules
find_package(PkgConfig)

pkg_check_modules(RSB rsb0.11)
pkg_check_modules(RSC rsc0.11)
pkg_check_modules(GLIB glib-2.0)
pkg_check_modules(PROTO protobuf)
pkg_check_modules(GSTR gstreamer-0.10)
find_package( OpenCV REQUIRED )


execute_process(COMMAND echo COMMAND ${CMAKE_CXX_COMPILER} -Wp,-v -x c++ - -fsyntax-only ERROR_VARIABLE GXX_OUTPUT)
set(ENV{GXX_OUTPUT} ${GXX_OUTPUT})
execute_process(COMMAND echo ${GXX_OUTPUT} COMMAND grep "^\ " COMMAND sed "s#\ ##g" COMMAND tr "\n" "\\;" OUTPUT_VARIABLE GXX_INCLUDES) 

# Set the include pathe
include_directories(
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    ${GLIB_INCLUDE_DIRS}                                            # glib
    ${GSTR_INCLUDE_DIRS}                                            # gstreamer
    ${PROTO_INCLUDE_DIRS}                                           # Protobuffer
    "../../includes"                                                # Other includes like converters and stuff
    "$ENV{CVB}/include"                                               # Stemmer imaging
    "${CMAKE_CURRENT_BINARY_DIR}"                                   # add the binary tree to the search path for include files
    "$ENV{SDKTARGETSYSROOT}/usr/include"                            # System includes
    "$ENV{SDKTARGETSYSROOT}/usr/include/c++"                        # System c++ includes
    "$ENV{SDKTARGETSYSROOT}/usr/include/c++/arm-poky-linux-gnueabi" # IO streams
    "../ARToolKit/include"                                          # ARToolKit
    ${GXX_INCLUDES}
)

# add the executable
add_executable ("${PROJECT_NAME}" main.cxx object.cpp ../../includes/types/twbTracking.pb.cc)

# Set the compiler
# SET (CMAKE_C_COMPILER $ENV{CPP})
# SET (CMAKE_CXX_COMPILER $ENV{CXX})

# Link the executable to the library. 
# SET (BLAH "$ENV{SDKTARGETSYSROOT}/usr/lib")
link_directories (
    "$ENV{SDKTARGETSYSROOT}/usr/lib"
    "$ENV{CVB}/lib"                                             # Stemmer imaging
    "${CMAKE_CURRENT_BINARY_DIR}/../ARToolKit/lib"              # ARToolKit
)

add_definitions (
)

target_link_libraries (
    "${PROJECT_NAME}"
    rsb0.11
    boost_system
    boost_signals
    boost_program_options
    rsc0.11
    ${OpenCV_LIBS}
    # -- stemmer imaging
    :"$ENV{CVB}/lib/libCVCImg.so"
    :"$ENV{CVB}/lib/libCVCUtilities.so"
    :"$ENV{CVB}/lib/libCVCDriver.so"
    :"$ENV{CVB}/lib/libCVGenApi.so"
    #-lopencv_core -lopencv_highgui -lopencv_imgproc
    # -- openGL and glut
    GL
    glut
    # -- glib
    ${GLIB_LIBRARIES}
    # -- ARToolKit
    ${CMAKE_CURRENT_BINARY_DIR}/../ARToolKit/lib/libAR.a
    ${CMAKE_CURRENT_BINARY_DIR}/../ARToolKit/lib/libARgsub.a
    ${CMAKE_CURRENT_BINARY_DIR}/../ARToolKit/lib/libARgsubUtil.a
    ${CMAKE_CURRENT_BINARY_DIR}/../ARToolKit/lib/libARMulti.a
    ${CMAKE_CURRENT_BINARY_DIR}/../ARToolKit/lib/libARvideo.a
    # -- g-streamer
    ${GSTR_LIBRARIES}
    # -- Protobuffer
    ${PROTO_LIBRARIES}
)

message (STATUS  "${GLIB_LIBRARIES}")

# Get rid of depricated warnings
SET (
    CMAKE_CXX_FLAGS
    "-Wno-deprecated-declarations -Wno-write-strings -O2"
)
