# Workaround to find Boost in the cross compiling environment
# Example usage:
# # Set the boost components as discribed in http://www.cmake.org/cmake/help/v3.0/module/FindBoost.html
# SET (BOOST_COMPONENTS regex date_time program_options system thread)
# include(findPackageBoost)

# First try to find boost via the normal FIND_PACKAGE command 
FIND_PACKAGE(Boost QUIET COMPONENTS "${BOOST_COMPONENTS}")

# If this does not work, define it via the cross compiler environment
IF (NOT Boost_FOUND)
  SET(Boost_LIBRARIES "")
  MESSAGE(STATUS "Boost module NOT found, using the cross compiler env. in $ENV{SDKTARGETSYSROOT}")
  SET(Boost_FOUND "TRUE")
  FOREACH (BOOST_COMP ${BOOST_COMPONENTS})
   SET(Boost_LIBRARIES "${Boost_LIBRARIES};-lboost_${BOOST_COMP}")
  ENDFOREACH ()
  SET(Boost_INCLUDE_DIR "$ENV{SDKTARGETSYSROOT}")
ENDIF()