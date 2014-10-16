cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

project ("${PROJECT_NAME}")

# Read the pkg-config modules
find_package(PkgConfig)


find_package(OpenCV REQUIRED)
find_package(RSB REQUIRED)
find_package(RSC REQUIRED)
find_package(RST REQUIRED)


# Set the include pathe
include_directories(
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    ${RST_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}                                          # OpenCV
    "../../includes"                                                # Other includes like converters and stuff
    "${CMAKE_CURRENT_BINARY_DIR}"                                   # add the binary tree to the search path for include files
    "$ENV{SDKTARGETSYSROOT}/usr/include"                            # System includes
    "$ENV{SDKTARGETSYSROOT}/usr/include/c++"                        # System c++ includes
    "$ENV{SDKTARGETSYSROOT}/usr/include/c++/arm-poky-linux-gnueabi" # IO streams
)

# Add folders to build and run CMakeLists.txt in it
add_subdirectory (../../includes/converter/matConverter lib/matConverter) 
add_subdirectory (../../includes/converter/vecIntConverter lib/vecIntConverter) 

# add the executable
add_executable ("${PROJECT_NAME}" main.cxx)

# Link the executable to the library.
link_directories (
    "$ENV{SDKTARGETSYSROOT}/usr/lib"
)

message (STATUS "LibPath ${CMAKE_LIBRARY_PATH}")

target_link_libraries (
    "${PROJECT_NAME}"
    # -- rsb
    ${RSB_LIBRARIES}
    # :librsb0.11.so.0.11
    ${RSC_LIBRARIES}
    ${OpenCV_LIBRARIES}
    # -- boost
    boost_system
    boost_thread
    boost_regex
    boost_filesystem
    pthread
    # -- Own libraries
    matConverter
    vecIntConverter
)

# Get rid of depricated warnings

SET (
    CMAKE_CXX_FLAGS
    "-Wall -Wno-deprecated-declarations -O2"
)

set( CMAKE_VERBOSE_MAKEFILE on )
