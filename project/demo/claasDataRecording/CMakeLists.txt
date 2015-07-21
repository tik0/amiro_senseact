cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

# Add folders to build and run CMakeLists.txt in it
add_subdirectory ($ENV{MUROX_PROJECT}/tools/GigEVision2RSB GigEVision2RSB)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseHokuyo senseHokuyo)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseLeuzeODS9ODS96B senseLeuzeODS9ODS96B)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/sensePepperlFuchsOMD8000 sensePepperlFuchsOMD8000)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseOpenNi2 senseOpenNi2)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseSick_LD_MRS senseSick_LD_MRS)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseLASE_2000D_226 senseLASE_2000D_226)
add_subdirectory ($ENV{MUROX_PROJECT}/tools/PeakCan PeakCan)
add_subdirectory ($ENV{MUROX_PROJECT}/tools/rsb_ws_bridge_claas rsb_ws_bridge_claas)
add_subdirectory ($ENV{MUROX_PROJECT}/sense/senseS10Terminal senseS10Terminal)
add_subdirectory ($ENV{MUROX_PROJECT}/tools/MachineModel MachineModel)
add_subdirectory ($ENV{MUROX_PROJECT}/tools/DisplayRoi DisplayRoi)

SET(folders "sensePepperlFuchsOMD8000;senseLeuzeODS9ODS96B;senseHokuyo;GigEVision2RSB;senseOpenNi2;senseSick_LD_MRS;senseLASE_2000D_226;PeakCan;rsb_ws_bridge_claas;senseS10Terminal;MachineModel;DisplayRoi")
MESSAGE( STATUS "Processing: " )
FOREACH(folder ${folders})
  INSTALL(FILES "${CMAKE_CURRENT_BINARY_DIR}/${folder}/${folder}"
  PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ GROUP_WRITE WORLD_READ OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
  DESTINATION "bin")
ENDFOREACH()
