cmake_minimum_required(VERSION 2.8.3)
project(natnet_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
  tf_conversions
  message_generation
)

add_message_files(
  FILES
  RigidBody.msg
  Marker.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES natnet_ros
  CATKIN_DEPENDS message_runtime geometry_msgs roscpp std_msgs tf tf_conversions
#   DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(natnet_node src/natnet_node.cpp)
target_link_libraries(natnet_node ${catkin_LIBRARIES})
add_dependencies(natnet_node natnet_ros_generate_messages_cpp)