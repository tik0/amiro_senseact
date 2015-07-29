cmake_minimum_required(VERSION 2.8)

# SDK project itself (without demos)
project(ibeosdk)

# run prebuild script for windows
#######################################################
if(WIN32)
	execute_process(COMMAND generateDataBlockList.bat
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()


#take all source and header files
#######################################################
file(GLOB_RECURSE SDK_SRCS *.cpp)
file(GLOB_RECURSE SDK_HDRS *.hpp)

# include directories
#######################################################
include_directories(${CMAKE_SOURCE_DIR}/src)

# find boost and include stuff
#######################################################
if(${MSVC})
set(Boost_USE_STATIC_LIBS ON)
endif()

find_package(Boost REQUIRED COMPONENTS system chrono regex thread program_options date_time)
include_directories(${Boost_INCLUDE_DIRS})

# targets
#######################################################

if(UNIX)
	add_library(${PROJECT_NAME} SHARED ${SDK_SRCS} ${SDK_HDRS})
	set_target_properties(${PROJECT_NAME} PROPERTIES
		VERSION ${IBEOSDK_VERSION}
		LIBRARY_OUTPUT_DIRECTORY ${IBEOSDK_LIBDIR})
	target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} -lrt)
endif()

#######################################################

if(${MSVC})
	# create filters for vs project
	foreach(f ${SDK_SRCS} ${SDK_HDRS})
		# path relative to ibeosdk folder
		file(RELATIVE_PATH rp ${CMAKE_CURRENT_SOURCE_DIR} ${f})
		get_filename_component(folder ${rp} DIRECTORY)

		set(p "FILES/${folder}")
		string(REPLACE / \\ p ${p})

		source_group(${p} FILES ${f})
	endforeach()


	# static on windows
	add_library(${PROJECT_NAME} STATIC ${SDK_SRCS} ${SDK_HDRS})
	set_target_properties(${PROJECT_NAME} PROPERTIES
		VERSION "${IBEOSDK_MAJOR_VERSION}.${IBEOSDK_MINOR_VERSION}.${IBEOSDK_PATCH_VERSION}"
		LIBRARY_OUTPUT_DIRECTORY ${IBEOSDK_LIBDIR})
	target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
	add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> ${IBEOSDK_LIBDIR})
endif()

#######################################################
