cmake_minimum_required(VERSION 3.0.2)
project(hexapod)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(Boost REQUIRED COMPONENTS thread)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  dynamixel_sdk
  message_generation
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  geometry_msgs roscpp dynamixel_sdk
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(hexapod src/config.cpp src/Hexapod_handler.cpp src/Main.cpp)
add_dependencies(hexapod ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hexapod ${catkin_LIBRARIES} -lwiringPi)