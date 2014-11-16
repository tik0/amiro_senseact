cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

project ("${PROJECT_NAME}")


# Read the pkg-config modules
find_package(PkgConfig)

pkg_check_modules(RSB rsb0.11)
pkg_check_modules(RSC rsc0.11)
pkg_check_modules(OPENCV opencv)
pkg_check_modules(LIBV4l2 libv4l2)

# Get the system includes
execute_process(COMMAND echo COMMAND ${CMAKE_CXX_COMPILER} -Wp,-v -x c++ - -fsyntax-only ERROR_VARIABLE GXX_OUTPUT)
set(ENV{GXX_OUTPUT} ${GXX_OUTPUT})
execute_process(COMMAND echo ${GXX_OUTPUT} COMMAND grep "^\ " COMMAND sed "s#\ ##g" COMMAND tr "\n" "\\;" OUTPUT_VARIABLE GXX_INCLUDES) 

# Set the include pathe
include_directories(
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    ${OPENCV_INCLUDE_DIRS}                                          # OpenCV
    ${LIBV4l2_INCLUDE_DIRS}                                         # Video for Linux
    "../../includes"                                                # Other includes like converters and stuff
    "${CMAKE_CURRENT_BINARY_DIR}"                                   # add the binary tree to the search path for include files
    ${GXX_INCLUDES}
)

# Add folders to build and run CMakeLists.txt in it
add_subdirectory (../../includes/converter/matConverter lib/matConverter)

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
    boost_regex
    boost_date_time
    boost_program_options
    boost_system
    boost_program_options
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
#     opencv_ts
    opencv_video
    opencv_videostab
    rt
    pthread
    m
    dl
    v4l2
    # -- Own libraries
    matConverter
)

# Get rid of depricated warnings
SET (
    CMAKE_CXX_FLAGS
    "-Wno-deprecated-declarations -O2"
)
