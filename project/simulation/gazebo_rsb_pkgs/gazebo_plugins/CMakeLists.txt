cmake_minimum_required(VERSION 2.8.3)
project(gazebo_plugins)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(XML libxml-2.0)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

# Depend on system install of Gazebo and SDFormat
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

# RSB environment
find_package(RSB REQUIRED)
find_package(RSC REQUIRED)
find_package(RST 0.11 REQUIRED COMPONENTS sandbox)
find_package(Protobuf REQUIRED)

INCLUDE_DIRECTORIES(BEFORE SYSTEM ${RST_INCLUDE_DIRS})

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${RSB_INCLUDE_DIRS}                                             # RSB
  ${RSC_INCLUDE_DIRS}                                             # RSC
  ${PROTOBUF_INCLUDE_DIRS}                                        # Protobuffer
)

ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_definitions("-fPIC -std=c++11") # what is this for?

## Plugins
add_library(gazebo_rsb_laser SHARED src/gazebo_rsb_laser.cpp)
target_link_libraries(gazebo_rsb_laser ${GAZEBO_LIBRARIES} ${PROTOBUF_LIBRARIES} ${RST_LIBRARIES} ${RSB_LIBRARIES} RayPlugin)

add_library(gazebo_rsb_wheel SHARED src/gazebo_rsb_wheel.cpp)
target_link_libraries(gazebo_rsb_wheel ${GAZEBO_LIBRARIES} ${PROTOBUF_LIBRARIES} ${RST_LIBRARIES} ${RSB_LIBRARIES})

##
## Add your new plugin here
##