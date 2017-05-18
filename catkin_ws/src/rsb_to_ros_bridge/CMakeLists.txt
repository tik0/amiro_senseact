cmake_minimum_required(VERSION 2.8.3)
project(rsb_to_ros_bridge)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sai_msgs
  sensor_msgs
  cv_bridge
  image_transport
  compressed_image_transport
)

find_package(PkgConfig)
pkg_check_modules(RSB rsb$ENV{MUROX_RSB_VERSION})
pkg_check_modules(RSC rsc$ENV{MUROX_RSB_VERSION})
FIND_PACKAGE(RST $ENV{MUROX_RSB_VERSION} REQUIRED COMPONENTS sandbox)
pkg_check_modules(PROTOBUF protobuf)
pkg_check_modules(OPENCV opencv)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rsb_to_ros_bridge
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${RSB_INCLUDE_DIRS}
  ${RST_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
)

LINK_DIRECTORIES (
  ${RSB_LIBRARY_DIRS}
  ${RSC_LIBRARY_DIRS}
  ${OPENCV_LIBRARY_DIRS}
)

add_executable(rst_vision_image_to_ros_sensormsgs_image
  src/rst_vision_image_to_ros_sensormsgs_image.cpp
)

target_link_libraries(rst_vision_image_to_ros_sensormsgs_image
  ${catkin_LIBRARIES}
  ${RSB_LIBRARIES}
  ${RST_LIBRARIES}
  ${RSTSANDBOX_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${OPENCV_LIBRARIES}
)

add_executable(rst_vision_laserscan_to_ros_sensormsgs_Laserscan
  src/rst_vision_laserscan_to_ros_sensormsgs_Laserscan.cpp
)

target_link_libraries(rst_vision_laserscan_to_ros_sensormsgs_Laserscan
  ${catkin_LIBRARIES}
  ${RSB_LIBRARIES}
  ${RST_LIBRARIES}
  ${RSTSANDBOX_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${OPENCV_LIBRARIES}
)

add_executable(rst_value_array_to_ros_int_array
  src/rst_value_array_to_ros_int_array.cpp
)

target_link_libraries(rst_value_array_to_ros_int_array
  ${catkin_LIBRARIES}
  ${RSB_LIBRARIES}
  ${RST_LIBRARIES}
  ${RSTSANDBOX_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)

add_executable(rst_pose_to_ros_posestamped
  src/rst_pose_to_ros_posestamped.cpp
)

target_link_libraries(rst_pose_to_ros_posestamped
  ${catkin_LIBRARIES}
  ${RSB_LIBRARIES}
  ${RST_LIBRARIES}
  ${RSTSANDBOX_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)

ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})

add_dependencies(rst_vision_image_to_ros_sensormsgs_image sai_msgs_gencpp)
add_dependencies(rst_value_array_to_ros_int_array sai_msgs_gencpp)
