cmake_minimum_required(VERSION 2.8.3)
project(dynamic_transform)

find_package(catkin REQUIRED COMPONENTS
  compressed_image_transport
  cv_bridge
  image_transport
  roscpp
  rospy
  sai_msgs
  sensor_msgs
  std_msgs
  tf
  rsb_to_ros_bridge
)

find_package(PkgConfig)
find_package(RSB 0.16 REQUIRED)
find_package(RSC 0.16 REQUIRED)
find_package(RST 0.16 REQUIRED COMPONENTS sandbox)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dynamic_transform
#  CATKIN_DEPENDS compressed_image_transport cv_bridge image_transport roscpp rospy sai_msgs sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${RSB_INCLUDE_DIRS}
  ${RST_INCLUDE_DIRS}
)

add_executable(dynamic_tf_with_odom
  src/dynamic_tf_with_odom.cpp
)

target_link_libraries(dynamic_tf_with_odom
  ${catkin_LIBRARIES}
)
