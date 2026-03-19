#include "builtin_interfaces/msg/duration.hpp"
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/byte.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>

#include "krabi_msgs/msg/ax12_info.hpp"
#include "krabi_msgs/msg/caisses_sides.hpp"
#include "krabi_msgs/msg/infos_stepper.hpp"
#include "krabi_msgs/msg/servos_cmd.hpp"
#include "krabi_msgs/msg/strat_movement.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/subscription.hpp"
#include <memory>

void create_subscriptions(
  rclcpp::Subscription<builtin_interfaces::msg::Duration>::SharedPtr& a_remaining_time_match_sub,
  std::function<void(const builtin_interfaces::msg::Duration::SharedPtr)>
    callback_remaining_time_match,
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr& a_tirette_sub,
  std::function<void(const std_msgs::msg::Bool::SharedPtr)> callback_tirette_sub,
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr& a_vacuum_sub,
  std::function<void(const std_msgs::msg::Float32::SharedPtr)> callback_vacuum_sub,
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr& a_other_robots_sub,
  std::function<void(const geometry_msgs::msg::PoseArray::SharedPtr)> callback_other_robots_sub,
  rclcpp::Subscription<krabi_msgs::msg::InfosStepper>::SharedPtr& a_elevator_sub,
  std::function<void(const krabi_msgs::msg::InfosStepper::SharedPtr)> callback_elevator_sub,
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
  rclcpp::Node* goal_node);