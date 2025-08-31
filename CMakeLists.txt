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
find_package(krabi_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
#find_package(krabilib REQUIRED)
#find_package(tf_conversions REQUIRED)
#find_package(eigen_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_library(krabilib
  src/krabilib/angle.cpp  
  src/krabilib/distance.cpp  
  src/krabilib/position.cpp  
  src/krabilib/pose.cpp  
)

add_library(krabilib_strat
  src/krabilib/strategie/dijkstra.cpp   
  src/krabilib/strategie/strategiev3.cpp
  src/krabilib/strategie/etape.cpp      
  src/krabilib/strategie/mediumLevelAction.cpp
  src/krabilib/strategie/actionGoTo.cpp   
  src/krabilib/strategie/aireDeConstruction.cpp
  src/krabilib/strategie/stockDeMatierePremiere.cpp
)

add_executable(goal_strategy_node 
  src/goal/main.cpp
  #src/goal/AX12.cpp
  src/goal/coupe2025.cpp
  src/goal/pump.cpp
  src/goal/servomotor.cpp
  src/goal/grabi.cpp
  src/goal/stepper.cpp
  src/goal/actuators2025.cpp
)

target_link_libraries(goal_strategy_node krabilib_strat krabilib )

ament_target_dependencies(goal_strategy_node 
  rclcpp
  geometry_msgs
  std_msgs 
  krabi_msgs
  visualization_msgs
  #eigen_conversions
  tf2_eigen#_conversions
  tf2_ros
  tf2_geometry_msgs
  #tf_conversions
  #sensor_msgs
)

ament_target_dependencies(krabilib_strat 
  rclcpp 
  tf2_eigen
  tf2_ros
  tf2_geometry_msgs
)


ament_target_dependencies(krabilib 
  rclcpp 
  tf2_eigen
  tf2_ros
  tf2_geometry_msgs
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/krabilib
  DESTINATION include)

install(TARGETS
  goal_strategy_node
  DESTINATION lib/${PROJECT_NAME}/
)

install(
  TARGETS krabilib
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)



ament_package()