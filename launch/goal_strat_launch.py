from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

def generate_launch_description():
    isBlue_value = LaunchConfiguration('isBlue')
    xRobotPos_value = LaunchConfiguration('xRobotPos')
    yRobotPos_value = LaunchConfiguration('yRobotPos')
    zRobotOrientation_value = LaunchConfiguration('zRobotOrientation')
    use_sim_time_value = LaunchConfiguration('use_sim_time')
    use_lidar_loc_value = LaunchConfiguration('use_lidar_loc')

    isBlue_launch_arg = DeclareLaunchArgument(
        'isBlue',
        default_value='False'
    )
    xRobotPos_launch_arg = DeclareLaunchArgument(
        'xRobotPos',
        default_value='1.25'
    )
    yRobotPos_launch_arg = DeclareLaunchArgument(
        'yRobotPos',
        default_value='0.5'
    )
    zRobotOrientation_launch_arg = DeclareLaunchArgument(
        'zRobotOrientation',
        default_value='0.0'
    )
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True'
    )
    use_lidar_loc_launch_arg = DeclareLaunchArgument(
        'use_lidar_loc',
        default_value='False'
    )
    # Zones de recalage et leurs séquences
    recalage_file_launch_arg = DeclareLaunchArgument(
        'recalage_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('goal_strategy'), 'config', 'recalage_2026.yaml'])
    )
    return LaunchDescription([
        recalage_file_launch_arg,
        use_lidar_loc_launch_arg,
        isBlue_launch_arg, xRobotPos_launch_arg, yRobotPos_launch_arg, zRobotOrientation_launch_arg,
        use_sim_time_launch_arg,
        launch_ros.actions.SetParameter(name='isBlue', value=isBlue_value),
        launch_ros.actions.SetParameter(name='xRobotPos', value=xRobotPos_value),
        launch_ros.actions.SetParameter(name='yRobotPos', value=yRobotPos_value),
        launch_ros.actions.SetParameter(name='zRobotOrientation', value=zRobotOrientation_value),
        launch_ros.actions.SetParameter(name='maxAngularSpeed', value="1"),
        Node(
            package='goal_strategy',
            namespace='krabi_ns',
            executable='goal_strategy_node',
            name='goal_strat',
            parameters=[{"use_sim_time": use_sim_time_value},
                        # Pose de départ, cible de go_to_start (même valeur que map_odom_publisher)
                        {"start_pose.x": ParameterValue(xRobotPos_value, value_type=float)},
                        {"start_pose.y": ParameterValue(yRobotPos_value, value_type=float)},
                        {"start_pose.theta": ParameterValue(zRobotOrientation_value, value_type=float)},
                        {"recalage.file": ParameterValue(LaunchConfiguration('recalage_file'), value_type=str)},
                        # Lidar loc : AMCL publie map->odom, recalage bordure désactivé
                        {"recalage.use_lidar_loc": ParameterValue(use_lidar_loc_value, value_type=bool)}],
            respawn=True,
            respawn_delay=5.0
            #,remappings=[
            #    ('/input/pose', '/turtlesim1/turtle1/pose'),
            #    ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            #]
        )
    ])