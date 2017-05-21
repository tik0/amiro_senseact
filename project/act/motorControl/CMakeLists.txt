cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

project ("${PROJECT_NAME}")


# Read the pkg-config modules
find_package(PkgConfig)
pkg_check_modules(PROTOBUF protobuf)
if ($ENV{ARCH} MATCHES "arm")
    pkg_check_modules(RSB rsb0.13)
    pkg_check_modules(RSC rsc0.13)
    pkg_check_modules(RST rstsandbox0.13)
else()
    pkg_check_modules(RSB rsb0.16)
    pkg_check_modules(RSC rsc0.16)
    pkg_check_modules(RST rstsandbox0.16)
endif()

# Set the include pathe
include_directories(
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    ${RST_INCLUDE_DIRS}                                             # RST
    ${PROTOBUF_INCLUDE_DIRS}                                        # Protobuffer
    "$ENV{MUROX_INCLUDE_DIRS}"                                                # Other includes like converters and stuff
    "${CMAKE_CURRENT_BINARY_DIR}"                                   # add the binary tree to the search path for include files
    "$ENV{SDKTARGETSYSROOT}/usr/include"                            # System includes
    "$ENV{SDKTARGETSYSROOT}/usr/include/c++"                        # System c++ includes
    "$ENV{SDKTARGETSYSROOT}/usr/include/c++/arm-poky-linux-gnueabi" # IO streams
)

# Add folders to build and run CMakeLists.txt in it

if (NOT TARGET vecIntConverter)
    add_subdirectory ($ENV{MUROX_INCLUDE_DIRS}/converter/vecIntConverter lib/vecIntConverter)
endif()
# add the executable
add_executable ("${PROJECT_NAME}" main.cxx)
# Link the executable to the library.
link_directories (
    "$ENV{SDKTARGETSYSROOT}/usr/lib"
)
target_link_libraries (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${RSTSANDBOX_LIBRARIES}
    boost_regex
    boost_date_time
    boost_program_options
    boost_system
    boost_thread
    #vecIntConverter
    pthread
)

# Get rid of depricated warnings

#SET (
#    CMAKE_CXX_FLAGS
#    "-Wno-deprecated-declarations -O2 -std=c++11 -Dold"
#)
ADD_DEFINITIONS (-Wall -Wno-deprecated-declarations -O2 -Wno-write-strings -std=c++11 -Dold)
ADD_DEFINITIONS(${RST_CFLAGS})
