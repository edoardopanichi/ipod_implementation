cmake_minimum_required(VERSION 2.8.3)
project(ipod_regulator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  geometry_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mav_regulator
  CATKIN_DEPENDS  roscpp
                  rospy
                  tf
                  std_msgs
                  geometry_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib
  src/ipod_regulator.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(ipod_regulator_node src/ipod_regulator_node.cpp)
target_link_libraries(ipod_regulator_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib
)
