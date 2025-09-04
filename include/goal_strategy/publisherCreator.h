
#include "krabi_msgs/msg/strat_movement.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rclcpp/node.hpp"
#include <memory>

void publisherCreator(
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr& a_is_blue_pub,
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& a_goal_pose_pub,
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& a_debug_ma_etapes_pub,
  rclcpp::Publisher<krabi_msgs::msg::StratMovement>::SharedPtr& a_strat_movement_pub,
  rclcpp::Node* goal_node);