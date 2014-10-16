cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

project ("${PROJECT_NAME}")

FIND_PACKAGE(OpenCV REQUIRED)

# Set the include pathe
include_directories(
    /usr/local/include                                       # For OpenCV
    /usr/include
    /usr/include/rsb0.9
    /usr/include/rsc0.9
    ../../includes                                              # Other includes like converters and stuff
    "${CMAKE_CURRENT_BINARY_DIR}"                            # add the binary tree to the search path for include files
)

# Add folders to build and run CMakeLists.txt in it
add_subdirectory (../../includes/converter/matConverter lib/matConverter) 

# add the executable
add_executable ("${PROJECT_NAME}" main.cxx)

# Link the executable to the library.
link_directories (
    "/usr/lib"                             # For standard libraries
    "/usr/local/lib/"                      # For OpenCV libraries
    "/usr/lib/x86_64-linux-gnu/"           # For boost libraries
)

target_link_libraries (
    "${PROJECT_NAME}"
    # -- boost
    boost_system
    boost_thread
    boost_regex
    boost_filesystem
    pthread
    # -- rsb
    :librsb.so.0.9
    :librsc0.9.so
#     :librsb0.11.so.0.11                                    # This is the explicit name to the rsb library
#     :librsc0.11.so.0.11                                 # This is the explicit name to the rsc library
    # -- OpenCV
#     opencv_imgproc
#     opencv_core
#     opencv_highgui
    # -- Own libraries
    matConverter
    "${OpenCV_LIBS}"
)

# Get rid of depricated warnings
SET (
    CMAKE_C_FLAGS
    -Wno-deprecated-declarations
)

SET (
    CMAKE_CXX_FLAGS
    -Wno-deprecated-declarations
)
