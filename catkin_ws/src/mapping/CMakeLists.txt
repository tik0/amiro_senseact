cmake_minimum_required(VERSION 2.8.3)
project(mapping)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cmake_modules
  laser_geometry
  tf
)

find_package(PkgConfig)
# find_package(RSB 0.16 REQUIRED)
# find_package(RSC 0.16 REQUIRED)
# find_package(RST 0.16 REQUIRED COMPONENTS sandbox)
# find_package(cmake_modules REQUIRED)
# find_package(Eigen REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mapping
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
#  ${RSB_INCLUDE_DIRS}
#  ${RST_INCLUDE_DIRS}
#  ${EIGEN_INCLUDE_DIRS}
)

LINK_DIRECTORIES (
#  ${RSB_LIBRARY_DIRS}
#  ${RSC_LIBRARY_DIRS}
)

add_executable(ros_laser_scan_to_ros_pointcloud2
  src/ros_laser_scan_to_ros_pointcloud2.cpp
)

target_link_libraries(ros_laser_scan_to_ros_pointcloud2
  ${catkin_LIBRARIES}
#  ${RSB_LIBRARIES}
#  ${RST_LIBRARIES}
#  ${RSTSANDBOX_LIBRARIES}
)

#ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})
