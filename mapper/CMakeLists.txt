cmake_minimum_required(VERSION 2.8.3)
project(mapper)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  nav_msgs
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

add_executable(${PROJECT_NAME} src/mapper.cpp
    src/main.cpp src/map_keeper.cpp src/map_builder.cpp)

target_link_libraries(${PROJECT_NAME}
${catkin_LIBRARIES}
)
