cmake_minimum_required(VERSION 3.0.2)
project(leo_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  map_msgs
  costmap_2d
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation
)

add_service_files(
  FILES
  CheckPoseCollision.srv
)

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs costmap_2d tf2_ros message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(footprint_collision_checker_node 
  src/footprint_collision_checker_node.cpp
)

add_dependencies(footprint_collision_checker_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(footprint_collision_checker_node
  ${catkin_LIBRARIES}
)

install(TARGETS footprint_collision_checker_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



