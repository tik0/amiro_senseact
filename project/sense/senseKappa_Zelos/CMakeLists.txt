cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")


# Get modules
find_package(Boost REQUIRED regex date_time program_options system thread)

INCLUDE_DIRECTORIES( 
	${Boost_INCLUDE_DIRS}
	include
	/usr/include/log4cxx
)
LINK_DIRECTORIES(../lib/x64_log4cxx)

set(source_files
 	src/SDK4SaveBuffer.cpp
 )

add_executable ("${PROJECT_NAME}" src/main.cpp ${source_files})


#SET(GCC_COVERAGE_LINK_FLAGS    "-lsdk4acquire -lsdk4control -lgevtl -lpthread -llog4cxx")
#SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )


target_link_libraries (
    "${PROJECT_NAME}"
    "sdk4acquire"
    "sdk4control"
    "gevtl"
    "pthread"
    "log4cxx"
     ${Boost_LIBRARIES}
)

#SET( COMPILE_FLAGS "-o3 -D_FILE_OFFSET_BITS=64 -D_LARGE_FILES -D_LOG4CXX_ -std=c++11" )
SET (CMAKE_CXX_FLAGS 
"-Wno-deprecated-declarations -Wno-write-strings -O3 -std=c++11 -D_FILE_OFFSET_BITS=64 -D_LARGE_FILES -D_LOG4CXX_"
)



