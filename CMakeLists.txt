cmake_minimum_required(VERSION 2.8.3)
project(goal_strategy)

add_compile_options(-std=c++11)

add_definitions(-D STANDALONE_STRATEGIE)
add_definitions(-D USE_ROS)
add_definitions(-D USE_IOSTREAM)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  krabi_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs krabi_msgs
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
src/goal/main.cpp
src/goal/coupe2019.cpp
src/Krabi/stm32/src/angle.cpp  src/Krabi/stm32/src/distance.cpp  src/Krabi/stm32/src/position.cpp  src/Krabi/stm32/src/positionPlusAngle.cpp  src/Krabi/stm32/src/vec2d.cpp  src/Krabi/stm32/src/vec3d.cpp
src/Krabi/stm32/src/strategie/abeille.cpp      src/Krabi/stm32/src/strategie/dijkstra.cpp   src/Krabi/stm32/src/strategie/strategiev3.cpp
src/Krabi/stm32/src/strategie/accelerator.cpp  src/Krabi/stm32/src/strategie/etape.cpp      src/Krabi/stm32/src/strategie/mediumLevelAction.cpp
src/Krabi/stm32/src/strategie/actionGoTo.cpp   src/Krabi/stm32/src/strategie/goldenium.cpp  src/Krabi/stm32/src/strategie/reservoirEau.cpp src/Krabi/stm32/src/strategie/port.cpp src/Krabi/stm32/src/strategie/phare.cpp src/Krabi/stm32/src/strategie/mancheAAir.cpp src/Krabi/stm32/src/strategie/mouillageNord.cpp src/Krabi/stm32/src/strategie/mouillageSud.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)