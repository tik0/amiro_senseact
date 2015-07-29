cmake_minimum_required(VERSION 2.8)
project(ibeosdk_demos)

# parse demo files
#######################################################

file(GLOB DEMO_SOURCES *.cpp)

# boost
#######################################################
include_directories(${CMAKE_SOURCE_DIR}/src)

find_package(Boost REQUIRED COMPONENTS system thread program_options date_time)
include_directories(${Boost_INCLUDE_DIRS})

# create demos
#######################################################
foreach(demo ${DEMO_SOURCES})

	if(${MSVC})
		source_group("FILES" FILES ${demo})
	endif()

	get_filename_component(demoName "${demo}" NAME_WE)
	if ( (${MSVC} STREQUAL "") OR (NOT ${demoName} STREQUAL "IbeoSdkDataLoggerDemo") )
		add_executable(${demoName} ${demo})
		target_link_libraries(${demoName} ibeosdk)

		# copy to bin folder
		add_custom_command(TARGET ${demoName} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${demoName}> ${IBEOSDK_BINDIR})
	endif()
endforeach()

#######################################################
