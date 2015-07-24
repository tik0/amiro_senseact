cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

SET( COMPILE_FLAGS "-o3 -D_FILE_OFFSET_BITS=64 -D_LARGE_FILES -D_LOG4CXX_" )
#SET(GCC_COVERAGE_LINK_FLAGS    "-lsdk4acquire -lsdk4control -lgevtl -lpthread -ldl")

INCLUDE_DIRECTORIES( 
	../../include
	../../lib/x64
)

set(source_files
 	SDK4SaveBuffer.cpp
 )

add_executable ("${PROJECT_NAME}" main.cpp ${source_files})

#SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )

target_link_libraries (
    "${PROJECT_NAME}"
    "-lsdk4acquire"
    "-lsdk4control"
    "-lgevtl"
    "-lpthread"
    "-llog4cxx"
)