cmake_minimum_required(VERSION 3.0.2)
project(hexapod)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

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

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  geometry_msgs roscpp dynamixel_sdk
)
include_directories(
  include/hexapod
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(hexbot src/config.cpp src/Hexapod_handler.cpp src/Main.cpp)
add_dependencies(hexbot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hexbot ${catkin_LIBRARIES} -lwiringPi -fsanitize=address -static-libasan -g)