cmake_minimum_required(VERSION 2.8.3)
project(natnet_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  roscpp
  std_msgs
  tf2_ros
)

add_message_files(
  FILES
    RigidBody.msg
    Marker.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES natnet_ros
  CATKIN_DEPENDS message_runtime geometry_msgs roscpp std_msgs tf2_ros
#   DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(natnet_node src/natnet_node.cpp)
target_link_libraries(natnet_node ${catkin_LIBRARIES})
add_dependencies(natnet_node natnet_ros_generate_messages_cpp)

add_executable(natnet_viz_node src/viz_node.cpp)
target_link_libraries(natnet_viz_node ${catkin_LIBRARIES})
add_dependencies(natnet_viz_node natnet_ros_generate_messages_cpp)