# Goal strategy of the Krabi robot

Chooses what to do: where and how to move (sends strat_movement to MainStrat), and what to do with the actuators (actuators2025)

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

### goal_pose

### debug_etapes

## is_blue


# Inputs

There are a lot of them, but nothing major

## Messages from the actuators

### stepper_info

### ax12_X_info

### digitalRead

### vacuum

## caisses_sides

2026-specific: how the caisses are oriented (from the camera)

## dynamic_obstacles
From lidar_strategy, but not limited to lidars

## Match state
### tirette

### remaining_time


