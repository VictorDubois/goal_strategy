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



add_executable(goal_strategy_node 
  src/goal/main.cpp
  src/goal/actuators.cpp
  src/goal/actuators2023.cpp
  src/goal/claws.cpp
  src/goal/coupe2021.cpp
  src/goal/coupe2022.cpp
  src/goal/coupe2023.cpp
  src/goal/grabber.cpp
  src/goal/pump.cpp
  src/goal/servomotor.cpp
  src/krabilib/angle.cpp  
  src/krabilib/distance.cpp  
  src/krabilib/position.cpp  
  src/krabilib/pose.cpp  
  src/krabilib/strategie/abeille.cpp
  src/krabilib/strategie/assiette.cpp
  src/krabilib/strategie/bouee.cpp
  src/krabilib/strategie/carreFouille.cpp
  src/krabilib/strategie/galerie.cpp
  src/krabilib/strategie/hexagon_plat.cpp
  src/krabilib/strategie/dijkstra.cpp   
  src/krabilib/strategie/pileGateau.cpp   
  src/krabilib/strategie/port.cpp   
  src/krabilib/strategie/strategiev3.cpp
  src/krabilib/strategie/accelerator.cpp  
  src/krabilib/strategie/etape.cpp      
  src/krabilib/strategie/mediumLevelAction.cpp
  src/krabilib/strategie/actionGoTo.cpp   
  src/krabilib/strategie/goldenium.cpp  
  src/krabilib/strategie/reservoirEau.cpp
  src/krabilib/strategie/vitrine.cpp
  src/krabilib/strategie/phare.cpp
  src/krabilib/strategie/mancheAAir.cpp
  src/krabilib/strategie/mouillageNord.cpp
  src/krabilib/strategie/mouillageSud.cpp
  src/krabilib/strategie/statuette.cpp
)



ament_target_dependencies(goal_strategy_node rclcpp geometry_msgs std_msgs 
krabi_msgs
visualization_msgs
#eigen_conversions
tf2_eigen#_conversions
tf2_ros
tf2_geometry_msgs
#tf_conversions
#sensor_msgs
)