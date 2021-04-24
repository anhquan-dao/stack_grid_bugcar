cmake_minimum_required(VERSION 3.0.2)

project(stack_grid_bugcar)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  move_base_msgs
  nav_msgs
  sensor_msgs
  rospy
  roscpp
  pluginlib
  costmap_2d
  tf
  tf2
  tf2_geometry_msgs
  message_generation
  # bug_diagnostics
)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   move_base_msgs
   nav_msgs 
   sensor_msgs
   
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME} src/stack_grid_bugcar.cpp src/simple_layer_obj.cpp src/single_layer_handler.cpp src/stack_grid_node.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

add_executable(
  stack_grid_node src/stack_grid.cpp
)
target_link_libraries(stack_grid_node ${PROJECT_NAME})
install(TARGETS stack_grid_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   
)