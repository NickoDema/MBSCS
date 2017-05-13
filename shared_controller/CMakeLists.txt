cmake_minimum_required(VERSION 2.8.3)
project(shared_controller)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  visualization_msgs
  message_generation
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mapper
#  CATKIN_DEPENDS nav_msgs roscpp sensor_msgs tf
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  src
  include
)

add_executable( ${PROJECT_NAME}_node 
                src/shared_controller.cpp
                src/shared_controller_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
${catkin_LIBRARIES}
)