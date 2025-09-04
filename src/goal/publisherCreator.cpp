#include "goal_strategy/publisherCreator.h"

void publisherCreator(
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr& a_is_blue_pub,
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& a_goal_pose_pub,
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& a_debug_ma_etapes_pub,
  rclcpp::Publisher<krabi_msgs::msg::StratMovement>::SharedPtr& a_strat_movement_pub,
  rclcpp::Node* goal_node)
{
    a_goal_pose_pub = goal_node->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 5);
    a_is_blue_pub = goal_node->create_publisher<std_msgs::msg::Bool>("is_blue", 5);

    a_debug_ma_etapes_pub
      = goal_node->create_publisher<visualization_msgs::msg::MarkerArray>("debug_etapes", 5);
    a_strat_movement_pub
      = goal_node->create_publisher<krabi_msgs::msg::StratMovement>("strat_movement", 5);
}