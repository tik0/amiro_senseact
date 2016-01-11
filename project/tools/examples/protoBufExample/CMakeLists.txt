cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

project ("${PROJECT_NAME}")

# Read the pkg-config modules
find_package(PkgConfig)

pkg_check_modules(RSB rsb0.11)
pkg_check_modules(RSC rsc0.11)
pkg_check_modules(RST_CONV rst-converters0.11)
pkg_check_modules(RST rst0.11)
pkg_check_modules(OPENCV opencv)

FIND_PACKAGE( Boost 1.54 COMPONENTS system program_options thread REQUIRED )

# Set the include pathe
include_directories(
    /usr/local/include                                       # For OpenCV
    /usr/include
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    ${RST_CONV_INCLUDE_DIRS}                                        # RST_CONV
    ${RST_INCLUDE_DIRS}                                             # RST
    ${OPENCV_INCLUDE_DIRS}                                          # OPENCV
    ../../includes                                              # Other includes like converters and stuff
    "${CMAKE_CURRENT_BINARY_DIR}"                            # add the binary tree to the search path for include files
    ${Boost_INCLUDE_DIR}
)

# add the executable
add_executable ("${PROJECT_NAME}" main.cxx)

# Link the executable to the library.
link_directories (
    "/usr/lib"                             # For standard libraries
    "/usr/local/lib/"                      # For OpenCV libraries
#     "/usr/lib/x86_64-linux-gnu/"           # For boost libraries
)

target_link_libraries (
    "${PROJECT_NAME}"
    # -- boost
    ${Boost_LIBRARIES}
    pthread
    # -- rsb
    rsb0.11
    rsc0.11
    #   OPENCV
    opencv_imgproc
    opencv_calib3d
    opencv_contrib
    opencv_core
    opencv_features2d
    opencv_flann
    opencv_gpu
    opencv_highgui
    opencv_legacy
    opencv_ml
    opencv_nonfree
    opencv_objdetect
    opencv_ocl
    opencv_photo
    opencv_stitching
    opencv_superres
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