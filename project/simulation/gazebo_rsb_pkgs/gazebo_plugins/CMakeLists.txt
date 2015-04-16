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
find_package( OpenCV REQUIRED )

INCLUDE_DIRECTORIES(BEFORE SYSTEM ${RST_INCLUDE_DIRS})

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${RSB_INCLUDE_DIRS}                                             # RSB
  ${RSC_INCLUDE_DIRS}                                             # RSC
  ${RST_INCLUDE_DIRS}                                             # RST
  ${PROTOBUF_INCLUDE_DIRS}                                        # Protobuffer
  "$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
)

ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_definitions("-fPIC -std=c++11 -O2")

## Plugins
add_library(gazebo_rsb_laser SHARED src/gazebo_rsb_laser.cpp $ENV{MUROX_INCLUDE_DIRS}/types/LocatedLaserScan.pb.cc)
target_link_libraries(gazebo_rsb_laser ${GAZEBO_LIBRARIES} ${PROTOBUF_LIBRARIES} ${RST_LIBRARIES} ${RSB_LIBRARIES} RayPlugin)

# add_library(gazebo_rsb_laser_inversemodel_edge SHARED src/gazebo_rsb_laser_inversemodel_edge.cpp $ENV{MUROX_INCLUDE_DIRS}/types/LocatedLaserScan.pb.cc  $ENV{MUROX_INCLUDE_DIRS}/cvplot/cvplot.cpp)
# target_link_libraries(gazebo_rsb_laser_inversemodel_edge ${GAZEBO_LIBRARIES} ${PROTOBUF_LIBRARIES} ${RST_LIBRARIES} ${RSB_LIBRARIES} RayPlugin ${OpenCV_LIBS})

# add_library(ClaasCan_CombineHarvester SHARED $ENV{MUROX_INCLUDE_DIRS}/types/ClaasCan_CombineHarvester.pb.cc)
# target_link_libraries(ClaasCan_CombineHarvester ${PROTOBUF_LIBRARIES})

add_library(gazebo_rsb_wheel SHARED src/gazebo_rsb_wheel.cpp)
target_link_libraries(gazebo_rsb_wheel ${GAZEBO_LIBRARIES} ${PROTOBUF_LIBRARIES} ${RST_LIBRARIES} ${RSB_LIBRARIES})

add_library(gazebo_rsb_gps SHARED src/gazebo_rsb_gps.cpp)
target_link_libraries(gazebo_rsb_gps ${GAZEBO_LIBRARIES} ${PROTOBUF_LIBRARIES} ${RST_LIBRARIES} ${RSB_LIBRARIES})

add_library(gazebo_rsb_odom_amiro SHARED src/gazebo_rsb_odom_amiro.cpp)
target_link_libraries(gazebo_rsb_odom_amiro ${GAZEBO_LIBRARIES} ${PROTOBUF_LIBRARIES} ${RST_LIBRARIES} ${RSB_LIBRARIES})

##
## Add your new plugin here
##
