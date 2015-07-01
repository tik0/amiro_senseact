cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

# Add folders to build and run CMakeLists.txt in it
add_subdirectory ($ENV{MUROX_PROJECT}/tools/GigEVision2RSB GigEVision2RSB)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseHokuyo senseHokuyo)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseLenzODS9ODS96B senseLenzODS9ODS96B)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/sensePepperlFuchsOMD8000 sensePepperlFuchsOMD8000)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseOpenNi2 senseOpenNi2)

SET(folders "sensePepperlFuchsOMD8000;senseLenzODS9ODS96B;senseHokuyo;GigEVision2RSB;senseOpenNi2")
MESSAGE( STATUS "Processing: " )
FOREACH(folder ${folders})
  INSTALL(FILES "${CMAKE_CURRENT_BINARY_DIR}/${folder}/${folder}"
  PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ GROUP_WRITE WORLD_READ OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
  DESTINATION "bin")
ENDFOREACH()