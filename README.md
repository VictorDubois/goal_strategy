# Goal strategy of the Krabi robot

Chooses what to do: where and how to move (sends strat_movement to MainStrat), and what to do with the actuators (actuators2025). For that, it uses a strategy engine, based on Dijkstra

# Overview of the strategy engine

The robot is allowed to move only on the edges of a graph, that is constructed by hand. The vertices ("sommets") of the graph are called "etapes". They represent either:
- a place where the robot can perform an action (ex: grab the caisses from a ZoneDeRamassage in 2026)
- or simply a "point de passage", where the robot has nothing to do in particular

The graph is fine-tuned, so that the robot does not collide with the static obstacles, and can efficiently move from one interresting place to another.

When the robot needs a new goal (start of the match, when the previous is done, or in case the previous is unreachable), the following happens:
- the  [Dijkstra's algorithm](https://fr.wikipedia.org/wiki/Algorithme_de_Dijkstra#Distance_entre_la_ville_A_et_la_ville_J) is ran on the current graph, from where the robot currently is
- that gives the robot the distances to go to all the etapes
- an simple heuristic uses that + the potential points awarded by the various etapes, to choose the next goal
- using the state of the etapes after running dijkstra, that gives the shortest path to follow

Notes:
- "but A* is faster than Dijkstra!" => No and I don't care :p
  - No, because A* only computes the path between two points, while we heuristic needs the distances to all the etapes
  - I don't care, because the graph is hand made, has only ~30 vertices, so the compute time is negligible (even when it cas done on a microcontroler)
- there are quite a lot of added functionnalities, such as handling situations were an etape is unreachable, having multiple etapes for one action (ex: the same caisses can be taken from several places), activating an etape only after completing another, etc.

<img width="1301" height="805" alt="rosgraph_editor_goalstrat" src="https://github.com/user-attachments/assets/9aab83fc-d9cc-45fe-bbb5-e99c101bc6aa" />

# Outputs

## strat_movement
Custom message, sent to [main_strategy](https://github.com/VictorDubois/main-strategy), to detail where and how the robot should move (type: krabi_msgs/msg/StratMovement)
- What the the goal's pose
- Should the robot go forward/backwards
- What is the absolute max speed
- What is the max speed when reaching the goal
- What should the mode be (rotation only, translatation only, recalage bordure... See full list in the message)
- Should the odometry be reset

## actuators2025
Custom message, sent to the [actuator board](https://github.com/VictorDubois/ActuatorBoardCode), via the [CAN broker](https://github.com/VictorDubois/krabi-can-broker) (type: [Actuators2025](https://github.com/VictorDubois/krabi-msgs/blob/48950764bd21f7d45e99eb80eac04f454b944c8c/msg/Actuators2025.msg))
Controls how the actuators should move (

## Debug
Those are not used by the robot, but helpful to be displayed in Rviz/FoxGlove

### goal_pose
Pose of the current goal (type: [PoseStamped](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html))

### debug_etapes
Visualization of the etapes (type: [MarkerArray](https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html))

- red lines: bidirectionnal edges of the graph on which the robot is allowed to move
- red lines (bigger): path the robot is taking to go to the next goal
- red arrows: unidirectionnal edges of the graph on which the robot is allowed to move
- black cubes: etapes disabled (either not allowed yet, or disabled because the robot has failed to reach it - usually because of an obstacle)
- other shapes : year-specific objectives (2026: yellow/blue = caisses de noisettes, white squares = gardes manger)

2026 (Winter is coming):

<img width="500" height="295" alt="image" src="https://github.com/user-attachments/assets/241cd9c7-f1fd-43be-b8cb-98a4e4b6732e" />

2025 (The show must go on):

<img width="500" height="350" alt="Capture d’écran du 2026-06-01 13-33-53" src="https://github.com/user-attachments/assets/1844b7b9-5be3-4495-9406-41f30b331fdc" />

2024 (Farming Mars):

<img width="500" height="350" alt="image" src="https://github.com/user-attachments/assets/6cce0cf6-6dbb-47ed-9c8c-f9fae3a13756" />

2015 (Robomovies):

<img width="500" height="328" alt="krabi_2015_graph" src="https://github.com/user-attachments/assets/12686546-18d8-43ee-8835-3ec320bc1aed" />


## is_blue
Team color, received from ROS2 params, forwarded as a topic (type: [Bool](https://docs.ros2.org/foxy/api/std_msgs/msg/Bool.html))

# Inputs

There are a lot of them, but nothing major

## Messages from the actuators

### stepper_info
State of the Stepper motor (custom type: [InfosStepper](https://github.com/VictorDubois/krabi-msgs/blob/feature/ROS2/msg/InfosStepper.msg))

### ax12_X_info
State of the AX12 Servomotor n°X (custom type: [AX12Info](https://github.com/VictorDubois/krabi-msgs/blob/feature/ROS2/msg/AX12Info.msg))

### digitalRead
State of the digital inputs of the Actuator board. Each bit is the state of one input (type: [Byte](https://docs.ros2.org/foxy/api/std_msgs/msg/Byte.html))

### vacuum
State of the vaccum in the suction cup circuit (type: [Float32](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html))

## caisses_sides
2026-specific: how are caisses oriented, from [krabi_caisse_detector](https://github.com/VictorDubois/krabi_caisse_detector) (custom type: [CaissesSides](https://github.com/VictorDubois/krabi-msgs/blob/feature/ROS2/msg/CaissesSides.msg))

## dynamic_obstacles
From lidar_strategy, the dynamic obstacles (usually other robots), in case we want to take them into account (type: [PoseArray](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseArray.html))

## Match state
### tirette
State of the tirette, that signals the start of the Match, read as a GPIO (type: [Bool](https://docs.ros2.org/foxy/api/std_msgs/msg/Bool.html))

### remaining_time
Time remaining until the robot has to stop. Note: it is not the actual end of the match, as the PAMIs can move after that. (type: [Duration](https://docs.ros2.org/foxy/api/builtin_interfaces/msg/Duration.html))

# Guide to the files structure
## goal
- main.cpp: the main file, hosts the GoalStrat class. Ideally, it should be split up (~1.6k lines)
- subscriptionCreator/publisherCreator: handle the creation of ROS2 publishers and subscribers (initially for compile time performance)
- coupe202X.cpp: describes the graph on which the robot moves. Uses the classes defined in the krabilib <= this changes every year
- actuators2025.cpp: handles sending the orders to the actuator PCB
- bilig.cpp/grabi.cpp: complexe actuators for specific years. Defines the sequences to perform the various actions (ex: flip the 2026 caisses for bilig)
- pump/servomotor/stepper.cpp: simple actuators

## krabilib
- angle, distance, pose, position.cpp: define values and the operators to manipulate them
- stragie/strategiev3.cpp, etape.cpp, djikstra.cpp, mediumLevelAction.cpp: graph engine, used by coupe202X.cpp
- stragie/gardeManger, zoneDeRamassage, etc: year-specific action to realize. Corresponds to a vertex in the graph

# What to do for a new year
A lot!
