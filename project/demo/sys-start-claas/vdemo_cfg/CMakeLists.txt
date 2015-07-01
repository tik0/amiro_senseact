CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

PROJECT(vdemo_scripts)

INCLUDE(FindPkgConfig)
PKG_SEARCH_MODULE(VDEMO REQUIRED vdemo)

SET(PROGNAME vdemo_scripts)

SET(vdemo_prefix ${VDEMO_PREFIX})
SET(vdemo_demoroot "${CMAKE_INSTALL_PREFIX}/etc/vdemo_scripts")

MESSAGE("vdemo_prefix: ${vdemo_prefix}")
MESSAGE("vdemo_demoroot: ${vdemo_demoroot}")

configure_file(robocup_defines.sh.in etc/robocup_defines.sh @ONLY )
configure_file(claas_defines.sh.in etc/claas_defines.sh @ONLY )

FILE(GLOB files "*.sh.in")
list(REMOVE_ITEM files "${CMAKE_CURRENT_SOURCE_DIR}/robocup_defines.sh.in")
list(REMOVE_ITEM files "${CMAKE_CURRENT_SOURCE_DIR}/claas_defines.sh.in")

# Remove all generated scripts from file list in case of in source builds
FOREACH(item ${files})
  IF(${item} MATCHES "/vdemo_[^/]*$")
    LIST(REMOVE_ITEM files ${item})
  ENDIF()
ENDFOREACH(item)

# configure demo files to inject variables
SET(demofiles)
MESSAGE( STATUS "Processing: " )
FOREACH(file ${files})
  GET_FILENAME_COMPONENT(demoname ${file} NAME_WE)   
  MESSAGE( STATUS "   ${demoname}" )
  LIST(APPEND demofiles ${demoname})
  CONFIGURE_FILE(${file} "etc/${demoname}.sh" @ONLY)
  INSTALL(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/${demoname}.sh" DESTINATION "etc/vdemo_scripts")
ENDFOREACH()

FILE(GLOB components "component_scripts/component_*")
LIST(REMOVE_ITEM components "*~")
INSTALL(FILES ${components}
  DESTINATION "etc/vdemo_scripts/component_scripts"
  PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
)
INSTALL(DIRECTORY "data" DESTINATION "etc/vdemo_scripts/")
INSTALL(DIRECTORY "machine_setup" DESTINATION "etc/vdemo_scripts/")
INSTALL(FILES "claas_base.sh" "${CMAKE_CURRENT_BINARY_DIR}/etc/claas_defines.sh" "component_default_claas_config.sh" "vdemo_aux_functions.sh" DESTINATION "etc/vdemo_scripts" PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ GROUP_EXECUTE WORLD_EXECUTE)
INSTALL(DIRECTORY "robot_setup" DESTINATION "etc/vdemo_scripts/")
INSTALL(FILES "robocup_base.sh" "${CMAKE_CURRENT_BINARY_DIR}/etc/robocup_defines.sh" "component_default_config.sh" "vdemo_aux_functions.sh" DESTINATION "etc/vdemo_scripts" PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ GROUP_EXECUTE WORLD_EXECUTE)

# configure starter script for each demo configuration
foreach(demofile ${demofiles})
  set (VDEMO_demofile "${vdemo_demoroot}/${demofile}.sh")
  SET(vdemo_starter "vdemo_${demofile}.sh")
  CONFIGURE_FILE(vdemo_start.in ${vdemo_starter})
  INSTALL(FILES "${PROJECT_BINARY_DIR}/${vdemo_starter}"
    DESTINATION bin
    PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ GROUP_WRITE WORLD_READ OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
  )
endforeach()
