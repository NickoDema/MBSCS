cmake_minimum_required(VERSION 2.8.3)
project(sd_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
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

add_executable(${PROJECT_NAME}_node src/sd_filter_node.cpp
               src/sd_filter.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

