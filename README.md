# Goal strategy of the Krabi robot

Chooses what to do: where and how to move (sends strat_movement to MainStrat), and what to do with the actuators (actuators2025)

<img width="1301" height="805" alt="rosgraph_editor_goalstrat" src="https://github.com/user-attachments/assets/9aab83fc-d9cc-45fe-bbb5-e99c101bc6aa" />

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

# Outputs

## strat_movement

## actuators2025

## Debug

### goal_pose

### debug_etapes

## is_blue
