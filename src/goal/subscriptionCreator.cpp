#include "goal_strategy/goal.h"

void GoalStrat::create_subscriptions(rclcpp::SubscriptionOptions l_sub_options)
{
    m_remaining_time_match_sub = this->create_subscription<builtin_interfaces::msg::Duration>(
      "/remaining_time",
      5,
      std::bind(&GoalStrat::updateRemainingTime, this, std::placeholders::_1),
      l_sub_options);

    m_tirette_sub = this->create_subscription<std_msgs::msg::Bool>(
      "tirette",
      5,
      std::bind(&GoalStrat::updateTirette, this, std::placeholders::_1),
      l_sub_options);

    m_vacuum_sub = this->create_subscription<std_msgs::msg::Float32>(
      "vacuum", 5, std::bind(&GoalStrat::updateVacuum, this, std::placeholders::_1), l_sub_options);

    m_ax12_1_info_sub = this->create_subscription<krabi_msgs::msg::AX12Info>(
      "ax12_1_info",
      5,
      std::bind(&GoalStrat::updateAX12Info1, this, std::placeholders::_1),
      l_sub_options);

    m_ax12_2_info_sub = this->create_subscription<krabi_msgs::msg::AX12Info>(
      "ax12_2_info",
      5,
      std::bind(&GoalStrat::updateAX12Info2, this, std::placeholders::_1),
      l_sub_options);
    m_ax12_3_info_sub = this->create_subscription<krabi_msgs::msg::AX12Info>(
      "ax12_3_info",
      5,
      std::bind(&GoalStrat::updateAX12Info3, this, std::placeholders::_1),
      l_sub_options);
    m_ax12_4_info_sub = this->create_subscription<krabi_msgs::msg::AX12Info>(
      "ax12_4_info",
      5,
      std::bind(&GoalStrat::updateAX12Info4, this, std::placeholders::_1),
      l_sub_options);
    m_digital_reads_sub = this->create_subscription<std_msgs::msg::Byte>(
      "digitalRead",
      5,
      std::bind(&GoalStrat::updateDigitalReads, this, std::placeholders::_1),
      l_sub_options);
    m_other_robots_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "dynamic_obstacles",
      5,
      std::bind(&GoalStrat::updateOtherRobots, this, std::placeholders::_1),
      l_sub_options);
}