#include "goal_strategy/subscriptionCreator.h"

void create_subscriptions(
  rclcpp::Subscription<builtin_interfaces::msg::Duration>::SharedPtr& a_remaining_time_match_sub,
  std::function<void(const builtin_interfaces::msg::Duration::SharedPtr)>
    callback_remaining_time_match,
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr& a_tirette_sub,
  std::function<void(const std_msgs::msg::Bool::SharedPtr)> callback_tirette_sub,
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr& a_recalage_sub,
  std::function<void(const std_msgs::msg::Bool::SharedPtr)> callback_recalage_sub,
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr& a_vacuum_sub,
  std::function<void(const std_msgs::msg::Float32::SharedPtr)> callback_vacuum_sub,
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr& a_other_robots_sub,
  std::function<void(const geometry_msgs::msg::PoseArray::SharedPtr)> callback_other_robots_sub,
  rclcpp::Subscription<krabi_msgs::msg::InfosStepper>::SharedPtr& /*a_elevator_sub*/,
  std::function<void(const krabi_msgs::msg::InfosStepper::SharedPtr)> /*callback_elevator_sub*/,
  rclcpp::Subscription<krabi_msgs::msg::AX12Info>::SharedPtr& a_ax12_1_info_sub,
  std::function<void(const krabi_msgs::msg::AX12Info::SharedPtr)> callback_ax12_1_info_sub,
  rclcpp::Subscription<krabi_msgs::msg::AX12Info>::SharedPtr& a_ax12_2_info_sub,
  std::function<void(const krabi_msgs::msg::AX12Info::SharedPtr)> callback_ax12_2_info_sub,
  rclcpp::Subscription<krabi_msgs::msg::AX12Info>::SharedPtr& a_ax12_3_info_sub,
  std::function<void(const krabi_msgs::msg::AX12Info::SharedPtr)> callback_ax12_3_info_sub,
  rclcpp::Subscription<krabi_msgs::msg::AX12Info>::SharedPtr& a_ax12_4_info_sub,
  std::function<void(const krabi_msgs::msg::AX12Info::SharedPtr)> callback_ax12_4_info_sub,
  rclcpp::Subscription<std_msgs::msg::Byte>::SharedPtr& a_digital_reads_sub,
  std::function<void(const std_msgs::msg::Byte::SharedPtr)> callback_digital_reads_sub,
  rclcpp::Subscription<krabi_msgs::msg::CaissesSides>::SharedPtr& a_caisses_sides_sub,
  std::function<void(const krabi_msgs::msg::CaissesSides::SharedPtr)> callback_caisses_sides_sub,
  rclcpp::SubscriptionOptions l_sub_options,
  rclcpp::Node* goal_node)
{
    a_remaining_time_match_sub = goal_node->create_subscription<builtin_interfaces::msg::Duration>(
      "/remaining_time", 5, callback_remaining_time_match, l_sub_options);
    a_tirette_sub = goal_node->create_subscription<std_msgs::msg::Bool>(
      "tirette", 5, callback_tirette_sub, l_sub_options);
    a_vacuum_sub = goal_node->create_subscription<std_msgs::msg::Float32>(
      "vacuum", 5, callback_vacuum_sub, l_sub_options);
    a_ax12_1_info_sub = goal_node->create_subscription<krabi_msgs::msg::AX12Info>(
      "ax12_1_info", 5, callback_ax12_1_info_sub, l_sub_options);
    a_ax12_2_info_sub = goal_node->create_subscription<krabi_msgs::msg::AX12Info>(
      "ax12_2_info", 5, callback_ax12_2_info_sub, l_sub_options);
    a_ax12_3_info_sub = goal_node->create_subscription<krabi_msgs::msg::AX12Info>(
      "ax12_3_info", 5, callback_ax12_3_info_sub, l_sub_options);
    a_ax12_4_info_sub = goal_node->create_subscription<krabi_msgs::msg::AX12Info>(
      "ax12_4_info", 5, callback_ax12_4_info_sub, l_sub_options);
    a_digital_reads_sub = goal_node->create_subscription<std_msgs::msg::Byte>(
      "digitalRead", 5, callback_digital_reads_sub, l_sub_options);
    a_other_robots_sub = goal_node->create_subscription<geometry_msgs::msg::PoseArray>(
      "dynamic_obstacles", 5, callback_other_robots_sub, l_sub_options);
    a_caisses_sides_sub = goal_node->create_subscription<krabi_msgs::msg::CaissesSides>(
      "caisses_sides", 5, callback_caisses_sides_sub, l_sub_options);

    a_recalage_sub = goal_node->create_subscription<std_msgs::msg::Bool>(
      "recalage", 5, callback_recalage_sub, l_sub_options);
}
