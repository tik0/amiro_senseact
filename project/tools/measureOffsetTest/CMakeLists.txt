cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

project ("${PROJECT_NAME}")


# Read the pkg-config modules
find_package(PkgConfig)

pkg_check_modules(RSB rsb0.11)
pkg_check_modules(RSC rsc0.11)

# Not needed. The libraries will be set manualy
# link_directories(
#     ${RSB_LIBRARY_DIRS}
#     ${OPENCV_LIBRARY_DIRS}
#     ${LIBV4l2_LIBRARY_DIRS}
# )

# Set the include pathe
include_directories(
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    ${OPENCV_INCLUDE_DIRS}                                          # OpenCV
    ${LIBV4l2_INCLUDE_DIRS}                                         # Video for Linux
    "$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
    "${CMAKE_CURRENT_BINARY_DIR}"                                   # add the binary tree to the search path for include files
    "$ENV{SDKTARGETSYSROOT}/usr/include"                            # System includes
    "$ENV{SDKTARGETSYSROOT}/usr/include/c++"                        # System c++ includes
    "$ENV{SDKTARGETSYSROOT}/usr/include/c++/arm-poky-linux-gnueabi" # IO streams
)

# Add folders to build and run CMakeLists.txt in it
add_subdirectory ($ENV{MUROX_INCLUDE_DIRS}/converter/vecIntConverter lib/vecIntConverter)

# add the executable
add_executable ("${PROJECT_NAME}" main.cxx $ENV{MUROX_INCLUDE_DIRS}/Color.cpp)

# Set the compiler
# SET (CMAKE_C_COMPILER $ENV{CPP})
# SET (CMAKE_CXX_COMPILER $ENV{CXX})

# Link the executable to the library. 
# SET (BLAH "$ENV{SDKTARGETSYSROOT}/usr/lib")
link_directories (
    "$ENV{SDKTARGETSYSROOT}/usr/lib"
)
target_link_libraries (
    "${PROJECT_NAME}"
    rsb0.11
    boost_regex-mt
    boost_date_time-mt
    boost_program_options-mt
    boost_system-mt
    boost_thread
    boost_signals
    rsc0.11
    # -- Own libraries
    vecIntConverter
)

# Additional compiler flags
ADD_DEFINITIONS (-Wall -Wno-deprecated-declarations -O2 -Wno-write-strings -std=c++11 -Wcpp)
# Special flags for RST
ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})
