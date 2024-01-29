cmake_minimum_required(VERSION 3.5)
project(goal_strategy)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-D STANDALONE_STRATEGIE)
add_definitions(-D USE_ROS)
add_definitions(-D USE_IOSTREAM)
#set(ROS_BUILD_TYPE Debug)


find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

include_directories(include)



add_executable(goal_strategy_node 
  src/goal/main.cpp
  src/goal/coupe2019.cpp
  src/Krabi/stm32/src/angle.cpp  
  src/Krabi/stm32/src/distance.cpp  
  src/Krabi/stm32/src/position.cpp  
  src/Krabi/stm32/src/positionPlusAngle.cpp  
  src/Krabi/stm32/src/vec2d.cpp  
  src/Krabi/stm32/src/vec3d.cpp
  src/Krabi/stm32/src/strategie/abeille.cpp
  src/Krabi/stm32/src/strategie/dijkstra.cpp   
  src/Krabi/stm32/src/strategie/strategiev3.cpp
  src/Krabi/stm32/src/strategie/accelerator.cpp  
  src/Krabi/stm32/src/strategie/etape.cpp      
  src/Krabi/stm32/src/strategie/mediumLevelAction.cpp
  src/Krabi/stm32/src/strategie/actionGoTo.cpp   
  src/Krabi/stm32/src/strategie/goldenium.cpp  
  src/Krabi/stm32/src/strategie/reservoirEau.cpp
)



ament_target_dependencies(goal_strategy_node rclcpp geometry_msgs std_msgs 
#sensor_msgs
)