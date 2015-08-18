cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

project ("${PROJECT_NAME}")

# Read the pkg-config modules
find_package(PkgConfig)
find_package(RSB)
find_package(RSC)
find_package(RST)

pkg_check_modules(PROTOBUF protobuf)

#find_library(CANINTERPRETER 
#libCANInterpreter.so
#../../includes
#)

# Get modules
#find_package(Boost REQUIRED regex date_time program_options system thread)

set(source_files
	../../includes/types/CanMessage.pb.cc
	#src/Odemetrie_data.cpp
	src/Odemetrie.cpp
	../../includes/types/MachineModel.pb.cc
)

# Set the include path
include_directories(
    ${Boost_INCLUDE_DIRS}
    ${RSB_INCLUDE_DIRS}
    ${RSC_INCLUDE_DIRS}
    ${RST_INCLUDE_DIRS}
    ${PROTO_INCLUDE_DIRS}
    "../../includes/types"
    "../../includes"
    "../CANInterpreter/src"
    "${CMAKE_CURRENT_BINARY_DIR}"
)

# add the executable
add_executable ("${PROJECT_NAME}" src/main.cpp ${source_files})

target_link_libraries (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}    
    ${Boost_LIBRARIES}
    ${PROTO_LIBRARIES}
    #${CANINTERPRETER}
)

# Additional compiler flags
ADD_DEFINITIONS (-Wall -Wno-deprecated-declarations -O2 -Wno-write-strings -std=c++11)
