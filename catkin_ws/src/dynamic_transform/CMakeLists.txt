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
)

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
)

add_executable(dynamic_tf_with_odom
  src/dynamic_tf_with_odom.cpp
)

target_link_libraries(dynamic_tf_with_odom
  ${catkin_LIBRARIES}
)
