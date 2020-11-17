cmake_minimum_required(VERSION 2.8.3)
project(goal_strategy)

add_compile_options(-std=c++11)

add_definitions(-D STANDALONE_STRATEGIE)
add_definitions(-D USE_ROS)
add_definitions(-D USE_IOSTREAM)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs
                                        krabi_msgs)
find_package(Eigen3 REQUIRED)
catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  krabilib
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  krabi_msgs
  DEPENDS
  EIGEN3)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})


add_executable(${PROJECT_NAME}_node src/goal/main.cpp src/goal/coupe2019.cpp)

file(GLOB_RECURSE krabilib_SRC "src/krabilib/*.cpp")

add_library(krabilib ${krabilib_SRC})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} krabilib)
