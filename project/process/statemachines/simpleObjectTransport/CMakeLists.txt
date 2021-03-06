cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

project ("${PROJECT_NAME}")


# Read the pkg-config modules
find_package(PkgConfig)

pkg_check_modules(RSB rsb0.13)
pkg_check_modules(RSC rsc0.13)
pkg_check_modules(RST rst0.13)
pkg_check_modules(RSTSANDBOX rstsandbox0.11)

# Need to add, because rst manual said so:
ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})

# Set the include pathe
include_directories(
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    ${RST_INCLUDE_DIRS}                                             # RST types
    ${RSTSANDBOX_INCLUDE_DIRS}                                      # RST syndbox types
    $ENV{MUROX_INCLUDE_DIRS}                                        # Other includes like converters and stuff
    "${CMAKE_CURRENT_BINARY_DIR}"                                   # add the binary tree to the search path for include files
    "$ENV{SDKTARGETSYSROOT}/usr/include"                            # System includes
    "$ENV{SDKTARGETSYSROOT}/usr/include/c++"                        # System c++ includes
    "$ENV{SDKTARGETSYSROOT}/usr/include/c++/arm-poky-linux-gnueabi" # IO streams
    ${GXX_INCLUDES}
)


message(STATUS " RST ${RST_INCLUDE_DIRS}")

if (NOT TARGET vecIntConverter)
    add_subdirectory ($ENV{MUROX_INCLUDE_DIRS}/converter/vecIntConverter lib/vecIntConverter)
endif()

file(GLOB SOURCES *.cxx)
file(GLOB HEADERS *.hpp)

# add the executable
add_executable ("${PROJECT_NAME}" ${SOURCES} ${HEADERS} $ENV{MUROX_INCLUDE_DIRS}/Color.cpp $ENV{MUROX_INCLUDE_DIRS}/extspread/extspread.hpp $ENV{MUROX_INCLUDE_DIRS}/extspread/extspread.cxx)

link_directories (
    "$ENV{SDKTARGETSYSROOT}/usr/lib"
)
target_link_libraries (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${RSTSANDBOX_LIBRARIES}
    # Boost
    boost_regex
    boost_date_time
    boost_program_options
    boost_system
    boost_thread
    # -- Own libraries
    vecIntConverter
    ${PROTO_LIBRARIES}
)

SET (
    CMAKE_CXX_FLAGS
    "-Wno-deprecated-declarations -O2 -std=c++0x"
)
