cmake_minimum_required(VERSION 3.0.2)
project(hexapod_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  message_generation
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(hexapod_control src/hexapod_control.cpp)
add_dependencies(hexapod_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hexapod_control ${catkin_LIBRARIES})